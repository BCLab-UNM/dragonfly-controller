#!/usr/bin/env python3
import argparse
import sys
import threading
import time
import rclpy

from datetime import datetime, timedelta
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, ParamSetV2
from rclpy.qos import QoSProfile, QoSHistoryPolicy, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterValue, ParameterType
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from rx.subject import Subject

from dragonfly_messages.msg import MissionStep, SemaphoreToken, CO2, PositionVector
from dragonfly_messages.srv import *
from .droneStreamFactory import DroneStreamFactory
from .actions import *
from .boundaryUtil import *
from .waypointUtil import *


class MissionStarter:
    start = False


class DragonflyCommand:
    TEST_ALTITUDE = 3.0

    def __init__(self, id, node):
        self.boundary_check_thread = None
        self.node = node
        self.logger = node.get_logger()
        self.zeroing = False
        self.canceled = False
        self.sincezero = datetime.now()
        self.id = id
        self.actionqueue = ActionQueue()
        self.mission_starter = MissionStarter()
        self.position = None
        self.local_position = None
        self.localposition = None
        self.orientation = None
        self.rtl_boundary = None
        self.max_altitude = 100

        self.stateSubject = Subject()
        self.local_position_observable = Subject()
        self.local_velocity_observable = Subject()
        self.dragonfly_announce_subject = Subject()
        self.semaphore_observable = Subject()

    def setmode(self, mode):
        self.logger.info(f"Set Mode {mode}")
        future = self.setmode_service.call_async(SetMode.Request(custom_mode=mode))

        def mode_finished(msg):
            result = future.result()
            self.logger.info(f"Set mode result: {result} {msg}")

        future.add_done_callback(mode_finished)

    def arm(self):
        self.logger.info("Arming")
        self.logger.info(self.arm_service.call(CommandBool.Request(value=True)))

        time.sleep(5)

    def disarm(self):
        self.logger.info("Disarming")
        self.logger.info(self.arm_service.call(CommandBool.Request(value=False)))

    def hello(self, request, response):
        self.logger.info("hello")
        return response

    def armcommand(self, request, response):
        self.logger.info("Commanded to arm")

        self.actionqueue.push(ModeAction(self.logger, self.setmode_service, "STABILIZE")) \
            .push(ArmAction(self.logger, self.logPublisher, self.arm_service)) \
            .push(SleepAction(self.logger, 5)) \
            .push(DisarmAction(self.logger, self.logPublisher, self.arm_service))

        return response

    def takeoff(self, request, response):
        self.logger.info("Commanded to takeoff")

        self.actionqueue.push(ArmedStateAction(self.logger, self.logPublisher, self.id, self.stateSubject)) \
            .push(ModeAction(self.logger, self.setmode_service, "STABILIZE")) \
            .push(ArmAction(self.logger, self.logPublisher, self.arm_service)) \
            .push(SleepAction(self.logger, 1)) \
            .push(ModeAction(self.logger, self.setmode_service, "GUIDED")) \
            .push(TakeoffAction(self.logger, self.logPublisher, self.takeoff_service, self.TEST_ALTITUDE))

        return response

    def land(self, request, response):
        self.logger.info("Commanded to land")

        self.actionqueue.push(LandAction(self.logger, self.logPublisher, self.land_service)) \
            .push(WaitForDisarmAction(self.logger, self.id, self.logPublisher, self.stateSubject)) \
            .push(ModeAction(self.logger, self.setmode_service, "STABILIZE"))

        return response

    def rtl_command(self, request, response):
        self.logger.info("Commanded to RTL")

        self.rtl()

        return response

    def rtl(self):
        self.cancel()

        self.setmode("RTL")
        self.logPublisher.publish(String(data="RTL"))

    def goto(self, request, response):
        self.logger.info("Commanded to goto")

        self.actionqueue.push(SetPositionAction(self.local_setposition_publisher, 0, 10, self.TEST_ALTITUDE, self.orientation)) \
            .push(SleepAction(self.logger, 10)) \
            .push(SetPositionAction(self.local_setposition_publisher, 0, 0, self.TEST_ALTITUDE, self.orientation)) \
            .push(SleepAction(self.logger, 10))

        return response

    def home(self, request, response):
        self.logger.info("Commanded to home")

        self.actionqueue.push(SetPositionAction(self.local_setposition_publisher, 0, 0, 10, self.orientation))

        return response

    def build_ddsa_waypoints(self, startingWaypoint, walk, stacks, swarm_size, swarm_index, loops, radius, step_length,
                             altitude, orientation):
        ddsaWaypoints = build3DDDSAWaypoints(Span(walk), stacks, swarm_size, swarm_index, loops, radius, step_length)

        localWaypoints = []
        for localwaypoint in ddsaWaypoints:
            localWaypoints.append(createWaypoint(
                startingWaypoint.x + localwaypoint.x,
                startingWaypoint.y + localwaypoint.y,
                altitude + localwaypoint.z,
                orientation
            ))

        return localWaypoints

    def build_ddsa(self, request, response):
        ddsaWaypoints = self.build_ddsa_waypoints(self.localposition, request.walk, request.stacks, 1, 0,
                                                  request.loops,
                                                  request.radius, request.step_length, request.altitude,
                                                  self.orientation)

        waypoints = []
        for localwaypoint in ddsaWaypoints:
            waypoints.append(createLatLon(localwaypoint.pose.position, self.localposition, self.position))

        return DDSAWaypoints.Response(waypoints=waypoints)

    def ddsa(self, request, response):
        self.logger.info("Commanded to ddsa")
        self.actionqueue.push(LogAction(self.logPublisher, "DDSA Started")) \
            .push(ModeAction(self.logger, self.setmode_service, 'GUIDED'))

        self.canceled = False

        waypoints = self.build_ddsa_waypoints(self.localposition, request.walk, request.stacks, 1, 0,
                                              request.loops,
                                              request.radius, request.step_length, request.altitude,
                                              self.orientation)

        self.runWaypoints("DDSA", waypoints, request.wait_time, request.distance_threshold)

        self.actionqueue.push(LogAction(self.logPublisher, "DDSA Finished"))

        return DDSA.Response(success=True, message=f"Commanded {self.id} to DDSA.")

    def build_lawnmower_waypoints(self, walk_boundary, boundary, walk, altitude, stacks, step_length, orientation):
        lawnmowerLocalWaypoints = []

        if walk_boundary:
            wrappedGPSBoundary = []
            wrappedGPSBoundary.extend(boundary)
            wrappedGPSBoundary.append(boundary[0])

            for waypoint in wrappedGPSBoundary:
                lawnmowerLocalWaypoints.append(
                    buildRelativeWaypoint(self.localposition, self.position, waypoint, altitude, self.orientation))

        lawnmowerLocalWaypoints.extend(
            build3DLawnmowerWaypoints(self.logger, Span(walk), altitude, self.localposition, self.position, stacks, boundary,
                                      step_length, orientation))

        return lawnmowerLocalWaypoints

    def build_lawnmower(self, request, response):

        waypoints = []
        for lawnmowerLocalWaypoint in self.build_lawnmower_waypoints(request.walk_boundary, request.boundary,
                                                                     request.walk, request.altitude,
                                                                     request.stacks, request.step_length,
                                                                     self.orientation):
            waypoints.append(createLatLon(lawnmowerLocalWaypoint.pose.position, self.localposition, self.position))

        return LawnmowerWaypoints.Response(waypoints=waypoints)

    def lawnmower(self, request, response):
        self.logger.info("Commanded to lawnmower")
        self.actionqueue.push(LogAction(self.logPublisher, "Lawnmower Started")) \
            .push(ModeAction(self.logger, self.setmode_service, 'GUIDED'))

        self.canceled = False

        waypoints = self.build_lawnmower_waypoints(request.walk_boundary, request.boundary, request.walk,
                                                   request.altitude, request.stacks, request.step_length,
                                                   self.orientation)

        self.runWaypoints("Lawnmower", waypoints, request.wait_time, request.distance_threshold)

        self.actionqueue.push(LogAction(self.logPublisher, "Lawnmower Finished"))
        return Lawnmower.Response(success=True, message=f"Commanded {self.id} to lawnmower.")

    def navigate(self, request, response):
        self.logger.info("Commanded to navigate")
        self.actionqueue.push(LogAction(self.logPublisher, "Navigation started")) \
            .push(ModeAction(self.logger, self.setmode_service, 'GUIDED'))

        self.canceled = False

        localWaypoints = []
        for waypoint in request.waypoints:
            localWaypoints.append(
                buildRelativeWaypoint(self.localposition, self.position, waypoint, waypoint.relative_altitude,
                                      self.orientation))

        self.runWaypoints("Navigation", localWaypoints, request.wait_time, request.distance_threshold)

        self.actionqueue.push(LogAction(self.logPublisher, "Navigation Finished"))

        return Navigation.Response(success=True, message=f"Commanded {self.id} to navigate.")

    def build_curtain_waypoints(self, startWaypoint, endWaypoint, altitude, stacks, stack_height, orientation):
        waypoints = []

        reverse = False
        for stack in range(stacks):
            if reverse:
                waypoints.append(createWaypoint(endWaypoint.x, endWaypoint.y, altitude + (stack_height * stack), orientation))
                waypoints.append(createWaypoint(startWaypoint.x, startWaypoint.y, altitude + (stack_height * stack), orientation))
            else:
                waypoints.append(createWaypoint(startWaypoint.x, startWaypoint.y, altitude + (stack_height * stack), orientation))
                waypoints.append(createWaypoint(endWaypoint.x, endWaypoint.y, altitude + (stack_height * stack), orientation))
            reverse = not reverse

        return waypoints

    def findWaypoint(self, waypoint_name, waypoints):
        for waypoint in waypoints:
            if waypoint.name == waypoint_name:
                return [buildRelativeWaypoint(self.localposition, self.position, waypoint, waypoint.relative_altitude,
                                              self.orientation), waypoint.distance_threshold]
        return [None, None]

    def findBoundary(self, boundary_name, boundaries):
        for boundary in boundaries:
            if boundary.name == boundary_name:
                return boundary.points
        return None

    def setup_drone(self, request, response):
        self.logger.info("Setup")

        self.canceled = False

        params = ParamSetV2.Request(
            param_id = 'RTL_ALT',
            value = ParameterValue(type = ParameterType.PARAMETER_INTEGER, integer_value = request.rtl_altitude))

        future = self.setparam_service.call_async(params)

        def mode_finished(msg):
            result = future.result()
            self.logger.info(f"Set param result: {result} {msg}")
            self.logPublisher.publish(String(data=f"Setup Success: {result.success}"))

        future.add_done_callback(mode_finished)

        self.rtl_boundary = request.rtl_boundary
        self.max_altitude = request.max_altitude

        return Setup.Response(success=True, message=f"Setup {self.id}")

    def rtl_boundary_check(self):
        rate = self.node.create_rate(10)
        while rclpy.ok():
            if self.localposition is not None and self.position is not None and not self.canceled:
                if self.localposition.z > self.max_altitude:
                    self.logPublisher.publish(String(data=f"Exceeded maximum altitude of {self.max_altitude}m"))
                    self.rtl()
                if self.rtl_boundary is not None and not isInside(self.position, self.rtl_boundary.points):
                    self.logPublisher.publish(String(data=
                        f"Exceeded RTL Boundary at {self.position.longitude}, {self.position.latitude}"))
                    self.rtl()

            rate.sleep()

    def mission(self, request, response):
        self.cancel()

        for step in request.steps:
            if step.msg_type == MissionStep.TYPE_START:
                self.logger.info("Start")
                self.actionqueue.push(MissionStartAction(self.logPublisher, self.mission_starter))
            elif step.msg_type == MissionStep.TYPE_TAKEOFF:
                self.logger.info("Takeoff")
                self.actionqueue.push(ArmedStateAction(self.logger, self.logPublisher, self.id, self.stateSubject)) \
                    .push(ModeAction(self.logger, self.setmode_service, "STABILIZE")) \
                    .push(ArmAction(self.logger, self.logPublisher, self.arm_service)) \
                    .push(SleepAction(self.logger, 3)) \
                    .push(ModeAction(self.logger, self.setmode_service, "GUIDED")) \
                    .push(TakeoffAction(self.logger, self.logPublisher, self.takeoff_service, step.takeoff_step.altitude)) \
                    .push(SleepAction(self.logger, 3))
            elif step.msg_type == MissionStep.TYPE_SLEEP:
                self.logger.info("Sleep")
                self.actionqueue.push(SleepAction(self.logger, step.sleep_step.duration))
            elif step.msg_type == MissionStep.TYPE_LAND:
                self.logger.info("Land")
                self.actionqueue.push(LandAction(self.logger, self.logPublisher, self.land_service)) \
                    .push(WaitForDisarmAction(self.logger, self.id, self.logPublisher, self.stateSubject)) \
                    .push(ModeAction(self.logger, self.setmode_service, "STABILIZE"))
            elif step.msg_type == MissionStep.TYPE_GOTO_WAYPOINT:
                self.logger.info("Waypoint")
                [waypoint, distance_threshold] = self.findWaypoint(step.goto_step.waypoint, request.waypoints)
                if waypoint is not None:
                    self.actionqueue.push(LogAction(self.logPublisher, f"Goto {step.goto_step.waypoint}")) \
                        .push(WaypointAction(self.logger, self.id, self.logPublisher, self.local_setposition_publisher, waypoint,
                                             distance_threshold, self.local_velocity_observable, self.local_setvelocity_publisher,
                                             self.local_position_observable, self.drone_stream_factory, self.pump_service,
                                             step.goto_step.run_pump, step.goto_step.pump_threshold, step.goto_step.pump_num))
            elif step.msg_type == MissionStep.TYPE_SEMAPHORE:
                self.logger.info("Semaphore")
                self.actionqueue.push(LogAction(self.logPublisher, "Waiting for semaphore...")) \
                    .push(SemaphoreAction(self.logger, self.id, step.semaphore_step.id, step.semaphore_step.drones, self.semaphore_publisher, self.semaphore_observable)) \
                    .push(LogAction(self.logPublisher, "Semaphore reached"))
            elif step.msg_type == MissionStep.TYPE_RTL:
                self.logger.info("RTL")
                self.actionqueue.push(LogAction(self.logPublisher, "RTL")) \
                    .push(ModeAction(self.logger, self.setmode_service, 'RTL')) \
                    .push(WaitForDisarmAction(self.logger, self.id, self.logPublisher, self.stateSubject))
            elif step.msg_type == MissionStep.TYPE_DDSA:
                self.logger.info("DDSA")
                self.actionqueue.push(LogAction(self.logPublisher, "DDSA"))
                [waypoint, distance_threshold] = self.findWaypoint(step.ddsa_step.waypoint, request.waypoints)
                if waypoint is not None:
                    waypoints = self.build_ddsa_waypoints(waypoint.pose.position, step.ddsa_step.walk, step.ddsa_step.stacks,
                                                          step.ddsa_step.swarm_size, step.ddsa_step.swarm_index,
                                                          step.ddsa_step.loops, step.ddsa_step.radius, step.ddsa_step.step_length,
                                                          step.ddsa_step.altitude, self.orientation)
                    self.actionqueue.push(AltitudeAction(self.id,
                                                         self.local_setposition_publisher,
                                                         waypoint.pose.position.z + (
                                                                 step.ddsa_step.radius * step.ddsa_step.swarm_index),
                                                         step.ddsa_step.distance_threshold,
                                                         self.local_position_observable))
                    position_waypoint = createWaypoint(
                        waypoint.pose.position.x - (step.ddsa_step.radius * step.ddsa_step.swarm_index),
                        waypoint.pose.position.y,
                        waypoint.pose.position.z + (step.ddsa_step.radius * step.ddsa_step.swarm_index),
                        self.orientation)
                    self.actionqueue.push(WaypointAction(self.logger, self.id, self.logPublisher, self.local_setposition_publisher, position_waypoint,
                                                         step.ddsa_step.distance_threshold, self.local_velocity_observable, self.local_setvelocity_publisher,
                                                         self.local_position_observable, self.drone_stream_factory, self.pump_service))
                    self.runWaypoints("DDSA", waypoints, step.ddsa_step.wait_time, step.ddsa_step.distance_threshold)
            elif step.msg_type == MissionStep.TYPE_LAWNMOWER:
                self.logger.info("Lawnmower")
                self.actionqueue.push(LogAction(self.logPublisher, "Lawnmower"))
                boundary = self.findBoundary(step.lawnmower_step.boundary, request.boundaries)
                if boundary is not None:
                    waypoints = self.build_lawnmower_waypoints(step.lawnmower_step.walk_boundary, boundary,
                                                               step.lawnmower_step.walk, step.lawnmower_step.altitude,
                                                               step.lawnmower_step.stacks, step.lawnmower_step.step_length,
                                                               self.orientation)
                    boundary_length = len(boundary) if step.lawnmower_step.walk_boundary else 0
                    self.runPlumeAwareWaypoints("Lawnmower",
                                                waypoints,
                                                boundary_length,
                                                step.lawnmower_step)
            elif step.msg_type == MissionStep.TYPE_NAVIGATION:
                self.logger.info("Navigation")
                self.actionqueue.push(LogAction(self.logPublisher, "Navigation"))
                localWaypoints = []
                for waypoint in step.navigation_step.waypoints:
                    localWaypoints.append(
                        buildRelativeWaypoint(self.localposition, self.position, waypoint, waypoint.relative_altitude,
                                              self.orientation))
                self.runWaypoints("Navigation", localWaypoints, step.navigation_step.wait_time,
                                  step.navigation_step.distance_threshold)
            elif step.msg_type == MissionStep.TYPE_FLOCK:
                self.logger.info("Flock")
                self.actionqueue.push(LogAction(self.logPublisher, "Flock")) \
                    .push(FlockingAction(self.logger, self.id, self.logPublisher, self.local_setvelocity_publisher, self.dragonfly_announce_subject,
                                         step.flock_step.x, step.flock_step.y, step.flock_step.leader, self.drone_stream_factory, self.semaphore_observable))
            elif step.msg_type == MissionStep.TYPE_GRADIENT:
                self.logger.info("Gradient")
                self.actionqueue.push(LogAction(self.logPublisher, "Following Gradient")) \
                    .push(
                    GradientAction(self.logger, self.id, self.logPublisher, self.local_setvelocity_publisher, step.gradient_step.drones,
                                   self.drone_stream_factory))
            elif step.msg_type == MissionStep.TYPE_CALIBRATE:
                self.logger.info("Calibration")
                self.actionqueue.push(LogAction(self.logPublisher, "Calibrating CO2")) \
                    .push(
                    CalibrateAction(self.logger, self.id, self.logPublisher, step.calibrate_step.drones, self.drone_stream_factory))
            elif step.msg_type == MissionStep.TYPE_CURTAIN:
                self.logger.info("Curtain")
                self.actionqueue.push(LogAction(self.logPublisher, "Curtain"))
                [startWaypoint, distance_threshold] = self.findWaypoint(step.curtain_step.start_waypoint,
                                                                        request.waypoints)
                [endWaypoint, distance_threshold] = self.findWaypoint(step.curtain_step.end_waypoint, request.waypoints)
                if startWaypoint is not None and endWaypoint is not None:
                    localWaypoints = self.build_curtain_waypoints(startWaypoint.pose.position,
                                                                  endWaypoint.pose.position,
                                                                  step.curtain_step.altitude,
                                                                  step.curtain_step.stacks,
                                                                  step.curtain_step.stack_height,
                                                                  self.orientation)
                    self.runPlumeAwareWaypoints("Lawnmower",
                                                localWaypoints,
                                                0,
                                                step.curtain_step)
            elif step.msg_type == MissionStep.TYPE_PUMP:
                self.logger.info("Pump")
                self.actionqueue.push(LogAction(self.logPublisher, "Pump")) \
                    .push(PumpAction(step.pump_step.pump_num, self.pump_service))
            elif step.msg_type == MissionStep.TYPE_SKETCH:
                self.logger.info("Sketch")
                self.actionqueue.push(LogAction(self.logPublisher, "Sketch")) \
                    .push(SketchAction(self.id, self.logPublisher, self.logger, self.local_setvelocity_publisher, self.dragonfly_announce_subject,
                                       step.sketch_step.offset, step.sketch_step.partner, step.sketch_step.leader, self.drone_stream_factory,
                                       self.semaphore_observable, self.semaphore_publisher, step.sketch_step.threshold))

            elif step.msg_type == MissionStep.TYPE_VERTICAL_TRANSECT:
                self.logger.info("Vertical Transect")
                [waypoint, distance_threshold] = self.findWaypoint(step.vertical_transect_step.waypoint, request.waypoints)
                max_altitude_waypoint = createWaypoint(waypoint.pose.position.x, waypoint.pose.position.y, step.vertical_transect_step.maximum_altitude, waypoint.pose.orientation)

                self.actionqueue.push(LogAction(self.logPublisher, f"Goto {step.vertical_transect_step.waypoint}")) \
                    .push(WaypointAction(self.logger, self.id, self.logPublisher, self.local_setposition_publisher, waypoint,
                                         distance_threshold, self.local_velocity_observable, self.local_setvelocity_publisher,
                                         self.local_position_observable, self.drone_stream_factory, self.pump_service)) \
                    .push(VerticalTransectDownAction(self.logger, self.id, self.logPublisher, self.drone_stream_factory, step.vertical_transect_step.minimum_altitude, self.local_setvelocity_publisher)) \
                    .push(LogAction(self.logPublisher, f"Goto maximum")) \
                    .push(WaypointAction(self.logger, self.id, self.logPublisher, self.local_setposition_publisher, max_altitude_waypoint,
                                         distance_threshold, self.local_velocity_observable, self.local_setvelocity_publisher,
                                         self.local_position_observable, self.drone_stream_factory, self.pump_service)) \
                    .push(LogAction(self.logPublisher, f"Reached maximum"))
            elif step.msg_type == MissionStep.TYPE_STOP_FLOCK:
                self.logger.info("Stop flocking")
                self.actionqueue.push(StopFlockingAction(self.logger, self.id, self.semaphore_publisher)) \
                    .push(LogAction(self.logPublisher, "Stopped flocking"))
            else:
                self.logger.info(f"Mission step not recognized: {step.msg_type}")

        self.actionqueue.push(LogAction(self.logPublisher, "Mission complete"))
        self.logPublisher.publish(String(data=f"Mission with {len(request.steps)} steps setup"))

        return Mission.Response(success=True, message=f"{self.id} mission received.")

    def start_mission(self, request, response):
        self.logger.info("Commanded to start mission")

        self.canceled = False
        self.mission_starter.start = True

        return response

    def runWaypoints(self, waypoints_name, waypoints, wait_time, distance_threshold):

        for i, waypoint in enumerate(waypoints):
            self.actionqueue.push(
                LogAction(self.logPublisher, f"Goto {waypoints_name} {i + 1}/{len(waypoints)}"))
            self.actionqueue.push(WaypointAction(self.logger, self.id, self.logPublisher, self.local_setposition_publisher, waypoint,
                                                 distance_threshold, self.local_velocity_observable, self.local_setvelocity_publisher,
                                                 self.local_position_observable, self.drone_stream_factory, self.pump_service))
            if wait_time > 0:
                self.actionqueue.push(SleepAction(self.logger, wait_time))
            self.actionqueue.push(WaitForZeroAction(self.logPublisher, self))

        return

    def runPlumeAwareWaypoints(self, name, waypoints, boundary_length, parameters):
        self.actionqueue.push(PlumeAwareLawnmowerAction(name, self.id, self.logPublisher, waypoints, boundary_length, parameters,
                                                        self.local_setposition_publisher, self.local_position_observable,
                                                        self.local_velocity_observable,
                                                        self.drone_stream_factory.get_drone(self.id).get_co2()))

    def flock(self, request, response):
        flockCommand = request.steps[0]  # @TODO: check if this is right
        self.actionqueue.push(ModeAction(self.logger, self.setmode_service, 'GUIDED')) \
            .push(FlockingAction(self.logger, self.id, self.logPublisher, self.local_setvelocity_publisher, flockCommand.x, flockCommand.y,
                           flockCommand.leader, self.node, self.drone_stream_factory, self.semaphore_observable))

        return Flock.Response(success=True, message=f"Flocking {self.id} with {flockCommand.leader}.")

    def position_callback(self, data):
        # print data
        self.position = data

    def localposition_callback(self, data):
        self.local_position_observable.on_next(data)
        self.localposition = data.pose.position
        self.orientation = data.pose.orientation

    def co2Callback(self, data):
        previous = self.zeroing
        self.zeroing = data.warming or data.zeroing
        if self.zeroing and not previous:
            self.logPublisher.publish(String(data='Zeroing'))
        elif not self.zeroing and previous:
            self.logPublisher.publish(String(data='Finished zeroing'))

    def cancelCommand(self, request, response):
        self.logger.info("Commanded to cancel")

        self.cancel()

        return response

    def cancel(self):
        self.canceled = True
        self.actionqueue.stop()
        self.actionqueue.push(
            StopInPlaceAction(self.logger, self.id, self.logPublisher, self.local_setposition_publisher, self.local_position_observable))

    def loop(self):
        try:
            rate = self.node.create_rate(1)
            while rclpy.ok():
                result = self.actionqueue.step()
                if not result == ActionState.WORKING:
                    self.actionqueue.stop()
                rate.sleep()
        except KeyboardInterrupt:
            self.logger.info("Shutting down...")

    def create_client_and_wait(self, type, name):
        client = self.node.create_client(type, name)
        # while not client.wait_for_service(timeout_sec=1.0):
        #    self.logger.info(f"{name} service not available, waiting again...")
        return client

    def create_command(self, name, callback, type=Simple):
        self.node.create_service(type, f"/{self.id}/command/{name}", callback)

    def setup(self):
        self.setparam_service = self.create_client_and_wait(ParamSetV2, f"/{self.id}/mavros/param/set")
        self.setmode_service = self.create_client_and_wait(SetMode, f"/{self.id}/mavros/set_mode")
        self.arm_service = self.create_client_and_wait(CommandBool, f"/{self.id}/mavros/cmd/arming")
        self.takeoff_service = self.create_client_and_wait(CommandTOL, f"/{self.id}/mavros/cmd/takeoff")
        self.land_service = self.create_client_and_wait(CommandTOL, f"/{self.id}/mavros/cmd/land")
        self.pump_service = self.create_client_and_wait(Pump, f"/{self.id}/pump")
        self.local_setposition_publisher = self.node.create_publisher(PoseStamped,
                                                                      f"/{self.id}/mavros/setpoint_position/local", qos_profile=QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=10))
        self.local_setvelocity_publisher = self.node.create_publisher(TwistStamped,
                                                                      f"/{self.id}/mavros/setpoint_velocity/cmd_vel", qos_profile=QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=10))
        # self.global_setpoint_publisher = self.node.create_publisher(GlobalPositionTarget, f"/{self.id}/mavros/setpoint_position/global", 10)

        # self.node.create_subscription(NavSatFix, f"/{self.id}/mavros/global_position/raw/fix", self.position_callback, 10)
        self.node.create_subscription(NavSatFix, f"/{self.id}/mavros/global_position/global",
                                      self.position_callback,
                                      qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.node.create_subscription(PoseStamped, f"/{self.id}/mavros/local_position/pose",
                                      self.localposition_callback,
                                      qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.node.create_subscription(CO2, f"/{self.id}/co2", self.co2Callback,
                                      qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.node.create_subscription(String, "/dragonfly/announce",
                                      lambda name: self.dragonfly_announce_subject.on_next(name),
                                      qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.semaphore_publisher = self.node.create_publisher(SemaphoreToken, "/dragonfly/semaphore",
                                                              qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))

        # Proactively create mavros publishers for dashboard rosbride avoid default RELIABLE, when it needs to be BEST_EFFORT
        self.node.create_publisher(NavSatFix, f"/{self.id}/mavros/global_position/global",
                                   qos_profile=QoSProfile(durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))
        self.node.create_publisher(PoseStamped, f"/{self.id}/mavros/local_position/pose",
                                   qos_profile=QoSProfile(durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))
        self.node.create_publisher(State, f"/{self.id}/mavros/gmavros/state",
                                   qos_profile=QoSProfile(durability=DurabilityPolicy.VOLATILE, reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))

        def updateStateSubject(state):
            self.stateSubject.on_next(state)

        self.node.create_subscription(State, f"{self.id}/mavros/state", updateStateSubject, 10)

        self.node.create_subscription(TwistStamped, f"{self.id}/mavros/local_position/velocity_local",
                                      lambda velocity: self.local_velocity_observable.on_next(velocity),
                                      qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.node.create_subscription(SemaphoreToken, "/dragonfly/semaphore",
                                      lambda token: self.semaphore_observable.on_next(token),
                                      qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.logPublisher = self.node.create_publisher(String, f"{self.id}/log", qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))

        self.create_command("arm", self.armcommand)
        self.create_command("takeoff", self.takeoff)
        self.create_command("land", self.land)
        self.create_command("rtl", self.rtl_command)
        self.create_command("home", self.home)
        self.create_command("goto", self.goto)
        self.create_command("ddsa", self.ddsa, DDSA)
        self.create_command("lawnmower", self.lawnmower, Lawnmower)
        self.create_command("navigate", self.navigate, Navigation)
        self.create_command("mission", self.mission, Mission)
        self.create_command("setup", self.setup_drone, Setup)
        self.create_command("start_mission", self.start_mission)
        self.create_command("flock", self.mission, Mission)
        self.create_command("cancel", self.cancelCommand)
        self.create_command("hello", self.hello)

        self.node.create_service(DDSAWaypoints, f"/{self.id}/build/ddsa", self.build_ddsa)
        self.node.create_service(LawnmowerWaypoints, f"/{self.id}/build/lawnmower", self.build_lawnmower)

        self.drone_stream_factory = DroneStreamFactory(self.node)

        self.logger.info("Setup complete")

        self.boundary_check_thread = threading.Thread(target=self.rtl_boundary_check)
        self.boundary_check_thread.start()


def main():
    rclpy.init(args=sys.argv)

    parser = argparse.ArgumentParser(description='Drone command service.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    node = rclpy.create_node(f"{args.id}_remote_service")

    command = DragonflyCommand(args.id, node)

    command.setup()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    command.loop()

    node.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
