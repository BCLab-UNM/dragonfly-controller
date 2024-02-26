#!/usr/bin/env python3
from rx.subject import Subject

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Range
from dragonfly_messages.msg import CO2, PositionVector
from rclpy.qos import QoSProfile, QoSHistoryPolicy, HistoryPolicy, ReliabilityPolicy

class DroneStream:

    def __init__(self, name, node):
        self.name = name
        self.node = node

        self.position_subject = Subject()
        self.position_subject_init = False
        self.co2_subject = Subject()
        self.co2_subject_init = False
        self.velocity_subject = Subject()
        self.velocity_subject_init = False
        self.rangefinder_subject = Subject()
        self.rangefinder_subject_init = False
        self.sketch_control = Subject()
        self.sketch_control_init = False
        self.mean = 0
        self.std_dev = 1

    def set_co2_statistics(self, mean, std_dev):
        self.mean = mean
        self.std_dev = std_dev

    def get_position(self):
        if not self.position_subject_init:
            self.node.create_subscription(NavSatFix, f"{self.name}/mavros/global_position/global",
                                          lambda position: self.position_subject.on_next(position),
                                          qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.position_subject_init = True

        return self.position_subject

    def get_co2(self):
        if not self.co2_subject_init:
            self.node.create_subscription(CO2, f"{self.name}/co2", lambda value: self.co2_subject.on_next(value),
                                          qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.co2_subject_init = True

        return self.co2_subject

    def get_velocity(self):
        if not self.velocity_subject_init:
            self.node.create_subscription(TwistStamped, f"{self.name}/mavros/local_position/velocity_local",
                                          lambda value: self.velocity_subject.on_next(value),
                                          qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.velocity_subject_init = True

        return self.velocity_subject

    def get_rangefinder(self):
        if not self.rangefinder_subject_init:
            self.node.create_subscription(Range, f"{self.name}/mavros/rangefinder/rangefinder",
                                          lambda value: self.rangefinder_subject.on_next(value),
                                          qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.rangefinder_subject_init = True

        return self.rangefinder_subject

    def get_sketch_control(self):
        if not self.sketch_control_init:
            self.node.create_subscription(PositionVector, f"{self.name}/sketch",
                                          lambda value: self.sketch_control.on_next(value),
                                          qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.sketch_control_init = True

        return self.sketch_control

    def get_sketch_control_publisher(self):
        return self.node.create_publisher(PositionVector, f"{self.name}/sketch",
                                          qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))

class DroneStreamFactory:

    def __init__(self, node):
        self.node = node

        self.drones = {}

    def get_drone(self, name):
        if name not in self.drones.keys():
            self.drones[name] = DroneStream(name, self.node)

        return self.drones[name]