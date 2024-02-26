#!/usr/bin/env python3
import math
import rx
import rx.operators as ops
import numpy as np
from sklearn.linear_model import LinearRegression

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from rx.subject import Subject
from dragonfly_messages.msg import LatLon, PositionVector
from dragonfly_messages.msg import SemaphoreToken

from .ActionState import ActionState

EARTH_CIRCUMFERENCE = 40008000

class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__

class ReadingPosition:

    def __init__(self, latitude, longitude, value):
        self.latitude = latitude
        self.longitude = longitude
        self.value = value

def calculate_angle(one, two):
    intermediate = ((one[0] * two[0]) + (one[1] * two[1])) / (np.linalg.norm(one) * np.linalg.norm(two))
    if intermediate > 1:
        intermediate = 1
    return math.acos(intermediate)

def difference_in_meters(one, two):
    return [
        ((one.longitude - two.longitude) * (EARTH_CIRCUMFERENCE / 360) * math.cos(one.latitude * 0.01745)),
        ((one.latitude - two.latitude) * (EARTH_CIRCUMFERENCE / 360))
    ]

def unitary(vector):
    vector_magnitude = np.linalg.norm(vector)
    if vector_magnitude == 0 and vector[0] == 0:
        return vector
    return [vector[0] / vector_magnitude, vector[1] / vector_magnitude]

def rotate_vector(vector, angle):
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                [np.sin(angle), np.cos(angle)]])
    return np.dot(rotation_matrix, vector)

def createLatLon(latitude, longitude):
    return LatLon(latitude=latitude, longitude=longitude, relative_altitude=0.0)

def average(one, two):
    return createLatLon((one.latitude + two.latitude) / 2, (one.longitude + two.longitude)/2)

def magnitude(vector):
    return np.linalg.norm(vector)

def calculate_turn_center(token):
    direction_vector = [token.x, token.y]
    rotated_direction_vector = rotate_vector(direction_vector, token.a)

    opposite = np.array(unitary(rotated_direction_vector)) * (10 / 2) # lambda / 2


    hypotenuse = np.array(unitary(rotate_vector(rotated_direction_vector, math.pi / 2))) * ((10 / 2) / math.tan(token.a / 2))

    # print(f"op: {opposite} hyp: {hypotenuse}")

    # hyp = 2 / np.arcsin(token.a)
    #
    # center_offset = rotate_vector([token.x, token.y], math.pi / 2)
    center_offset = opposite + hypotenuse

    longitude = token.position.longitude + ((center_offset[0]) / (math.cos(token.position.latitude * 0.01745) * (EARTH_CIRCUMFERENCE / 360)))
    latitude = token.position.latitude + ((center_offset[1]) / (EARTH_CIRCUMFERENCE / 360))
    center = createLatLon(latitude, longitude)

    return center

class SketchAction:
    SAMPLE_RATE = 0.1
    SKETCH_STARTING_THRESHOLD = 20 # m

    def __init__(self, id, log_publisher, logger, local_setvelocity_publisher, announce_stream, offset, partner, leader,
                 drone_stream_factory, semaphore_observable, semaphore_publisher, threshold):
        self.log_publisher = log_publisher
        self.logger = logger
        self.local_setvelocity_publisher = local_setvelocity_publisher
        self.id = id
        self.partner = partner
        self.offset = offset
        self.leader = leader
        self.threshold = threshold
        self.ros_subscriptions = []
        self.announce_stream = announce_stream
        self.drone_stream_factory = drone_stream_factory
        self.dragonfly_sketch_subject = Subject()
        self.semaphore_observable = semaphore_observable
        self.semaphore_publisher = semaphore_publisher

        self.navigate_subscription = rx.empty().subscribe()
        self.leader_broadcast_subscription = rx.empty().subscribe()
        self.receive_semaphore_subscription = rx.empty().subscribe()
        self.sketch_control_subscription = rx.empty().subscribe()

        self.target_position_vector = None
        self.encountered = False
        self.starting_position = None
        self.sketch_started = False
        self.position_reading_queue = []
        self.started = False

        self.status = ActionState.WORKING

    def navigate(self, input):

        twist = TwistStamped()

        for vector in input:
            twist.twist.linear.x += vector[0]
            twist.twist.linear.y += vector[1]

        self.local_setvelocity_publisher.publish(twist)

    def format_velocities(self, twist):
        return [
            twist.twist.linear.x,
            twist.twist.linear.y
        ]

    def step(self):
        if not self.started:
            self.started = True

            self.logger.info("Subscribing...")

            partner_drone = self.drone_stream_factory.get_drone(self.partner)
            self_drone = self.drone_stream_factory.get_drone(self.id)

            self.navigate_subscription = rx.combine_latest(
                partner_drone.get_position(),
                self_drone.get_position(),
                self.dragonfly_sketch_subject
            ).pipe(
                ops.sample(self.SAMPLE_RATE),
                ops.map(lambda positions: self.tandem(positions[0], positions[1], positions[2]))
            ).subscribe(lambda vectors: self.navigate(vectors))

            if self.leader:
                self.leader_broadcast_subscription = rx.combine_latest(
                    rx.just(self.get_leader_drone().get_sketch_control_publisher()),
                    self.setup_subject(self.partner),
                    self.setup_subject(self.id)
                ).subscribe(lambda position_reading_vector: self.broadcast_target(position_reading_vector[0], position_reading_vector[1], position_reading_vector[2]))
            else:
                self.sketch_control_subscription = self.get_leader_drone().get_sketch_control().subscribe(self.dragonfly_sketch_subject)
                self.receive_semaphore_subscription = self.semaphore_observable.subscribe(on_next = lambda token: self.listen_for_stop(token))

        return self.status

    @staticmethod
    def build_sketch_end_id(id):
        return f"sketch.{id}"

    def get_leader_drone(self):
        return self.drone_stream_factory.get_drone(self.id if self.leader else self.partner)

    def listen_for_stop(self, token):
        self.logger.info(f"{self.id}: Received token")
        if token.id == self.build_sketch_end_id(self.partner):
            self.end_sketch()

    def broadcast_end(self):
        self.logger.info(f"{self.id}: Publishing sketch stop")
        token = SemaphoreToken()
        token.drone = self.id
        token.id =  self.build_sketch_end_id(self.id)
        self.semaphore_publisher.publish(token)

    def end_sketch(self):
        self.status = ActionState.SUCCESS

    def setup_subject(self, drone):

        drone_streams = self.drone_stream_factory.get_drone(drone)

        position_subject = drone_streams.get_position()
        co2_subject = drone_streams.get_co2()

        return rx.combine_latest(position_subject, co2_subject).pipe(
            ops.map(lambda tuple, offset=drone_streams.mean: ReadingPosition(tuple[0].latitude, tuple[0].longitude, tuple[1].ppm - offset))
        )

    def tandem(self, partner_position, self_position, positionVector):

        tandem_offset = self.calculate_tandem_offset(partner_position, self_position)
        tandem_angle = self.calculate_tandem_angle(partner_position, self_position, positionVector)
        straight_line = self.calculate_straight_line(positionVector)
        turning = self.calculate_turn(partner_position, self_position, positionVector)
        offset_error_correction = self.calculate_error_correction(partner_position, self_position, positionVector)

        return [tandem_offset, tandem_angle, straight_line, turning, offset_error_correction]

    def calculate_tandem_offset(self, partner_position, self_position):
        distance_m = difference_in_meters(partner_position, self_position)
        offset_unitary = unitary(distance_m)

        position_offset = np.array(distance_m) - (np.array(offset_unitary) * self.offset)
        offset_magnitude = magnitude(position_offset)

        tandem_distance = position_offset * (2 / max(2, offset_magnitude))
        return tandem_distance

    def calculate_tandem_angle(self, partner_position, self_position, position_vector):
        if position_vector.movement == PositionVector.FORWARD:
            distance_m = difference_in_meters(self_position, partner_position)
            offset_unitary = unitary(distance_m)

            direction_vector = np.array([position_vector.x, position_vector.y])

            return -4 * (np.dot(offset_unitary, direction_vector)) * direction_vector
        else:
            return [0, 0]

    def calculate_straight_line(self, position_vector):
        if position_vector.movement == PositionVector.FORWARD:
            return [3 * position_vector.x, 3 * position_vector.y]
        else:
            return [0, 0]

    def calculate_turn(self, partner_position, self_position, position_vector):
        if position_vector.movement == PositionVector.TURN:

            center = calculate_turn_center(position_vector)

            self_center = difference_in_meters(center, self_position)
            self_center_distance = magnitude(self_center)
            partner_center_distance = magnitude(difference_in_meters(center, partner_position))

            base_velocity = 3

            if self_center_distance > partner_center_distance:
                velocity = base_velocity
            else:
                velocity = (self_center_distance / partner_center_distance) * base_velocity

            vector_to_center = unitary(self_center)

            starting_position = difference_in_meters(center, position_vector.position)
            angle = math.atan2(self_center[1], self_center[0]) - math.atan2(starting_position[1], starting_position[0])

            rotated_vector = rotate_vector([position_vector.x, position_vector.y], angle)

            forward_force = np.array(rotated_vector) * velocity
            centripetal_force = np.array(vector_to_center) * 5.5 * velocity / self_center_distance

            return forward_force + centripetal_force
        else:
            return [0, 0]

    def calculate_error_correction(self, partner_position, self_position, position_vector):
        if position_vector.movement == PositionVector.FORWARD:
            vector_to_target =  difference_in_meters(average(self_position, partner_position), position_vector.position)

            dot_product = np.dot([position_vector.x, position_vector.y], vector_to_target)

            vector_to_line = dot_product * np.array([position_vector.x, position_vector.y]) - vector_to_target

            magnitude_calc = magnitude(vector_to_line)

            max_magnitude = 0.1

            if magnitude_calc > max_magnitude:
                return vector_to_line / (magnitude_calc / max_magnitude)
            else:
                return vector_to_line
        else:
            # Maintain distance-to-center
            center = calculate_turn_center(position_vector) # position_vector.position ?
            expected_distance_to_center = magnitude(difference_in_meters(position_vector.position, center))

            average_to_center = difference_in_meters(average(self_position, partner_position), center)

            center_correction = unitary(average_to_center) * np.array(expected_distance_to_center - magnitude(average_to_center))

            return center_correction

    def broadcast_target(self, position_vector_publisher, partner_position, self_position):

        average_position = average(self_position, partner_position)

        self.position_reading_queue.append(partner_position)
        self.position_reading_queue.append(self_position)

        while len(self.position_reading_queue) > 200:
            self.position_reading_queue.pop(0)

        if not self.target_position_vector:
            position_vector = PositionVector()
            position_vector.movement = PositionVector.FORWARD
            position = createLatLon(average_position.latitude, average_position.longitude)

            position_vector.x = 0.0
            position_vector.y = 1.0
            position_vector.position = position
            position_vector.distance = 5.0

            self.target_position_vector = position_vector

        if not self.encountered and (self.inside(partner_position) or self.inside(self_position)):
            self.encountered = True
            self.starting_position = average_position
            self.log_publisher.publish(String(data="Encountered plume"))

        if self.encountered and not self.sketch_started and magnitude(difference_in_meters(self.starting_position, average_position)) > (2 * self.SKETCH_STARTING_THRESHOLD):
            self.sketch_started = True
            self.log_publisher.publish(String(data="Sketch started"))

        if self.sketch_started and magnitude(difference_in_meters(self.starting_position, average_position)) < self.SKETCH_STARTING_THRESHOLD:
            # We must be back to the start
            self.log_publisher.publish(String(data="Sketch ended where it started"))
            self.broadcast_end()
            self.end_sketch()

        if self.target_position_vector.movement == PositionVector.TURN and self.target_position_vector.a * self.target_position_vector.p > 4 * math.pi:
            # We are in a death spiral
            self.log_publisher.publish(String(data="Sketch ended from losing plume"))
            self.broadcast_end()
            self.end_sketch()

        if self.encountered and self.passed(average_position):
            self.target_position_vector = self.boundary_sketch(partner_position, self_position)
        # self.logger.info(f"PV: {self.target_position_vector} to {position_vector_publisher}")
        position_vector_publisher.publish(self.target_position_vector)
        self.dragonfly_sketch_subject.on_next(self.target_position_vector)

    def calculate_gradient(self):
        x = []
        y = []

        for reading_position in self.position_reading_queue:
            x.append([reading_position.longitude, reading_position.latitude])
            y.append(reading_position.value)

        x, y = np.array(x), np.array(y)

        model = LinearRegression().fit(x, y)

        return unitary([model.coef_[0], model.coef_[1]])

    # Algorithm 1 Ensures the robots are at distance √
    #
    # λ from each other and are oriented in the same direction.
    # Additional discussion with illustrative diagrams of this synchronization is in Appendix D.
    #
    # procedure SYNCHRONIZE(D1, D2)
    #   Path ← the polyline path of D2 from last crossing of BOUNDARY-SKETCH with the shape till current position.
    #   ∇ ← the gradient at the last boundary crossing for D2.
    #   L1 ← the line in the direction of ∇ through D1’s position.
    #   L2 ← the line in the direction of ∇ through D2’s position.
    #   if L1 crosses Path then
    #       Move D2 in its current direction until it is √λ distance away from L1.
    #       Change direction to ∇ and take a single step of length λ.
    #       Move D1 along L1 until it is √λ away from D2.
    #
    #   else
    #       Move D1 in its current direction until it is √ λ distance away from L2.
    #
    #   Change direction to ∇ and move until the distance from D2 is √λ.

    # Algorithm 2 Reestablishes “Sandwich” Invariant
    # procedure CROSS-BOUNDARY(D1, D2, α)
    #   p ← last position of D1 before crossing
    #   R ← the vertices of the regular polygon including D1’s position with exterior angle √λ and
    #       the edge beginning at D1’s position facing the direction of ∇ + α.
    #   P ← the vertices of the convex hull of R ∪ {p}. For all i : 0 ≤ i ≤ |P| − 1, let
    #       Pi be the i-th vertex in this convex hull, ordered such that P0 = p and P1 = D1’s current position.
    #   ∇ ← gradient at the last boundary crossing of D1
    #   i ← 1.
    #   while neither robot has crossed the boundary AND i + 1 < |P| do
    #       D1 moves to Pi+1.
    #       D2 moves to closest point from it that is √λ distance away from Pi and orthogonal to ∇ + iα
    #       i ← i + 1
    #   while neither robot has crossed the boundary do
    #       D1 moves towards point p taking steps of length λ.
    #       D2 moves to closest point from it that is √λ distance away from D1 and orthogonal to D1’s direction.
    #   if D2 crossed the boundary then
    #       SYNCHRONIZE (D1, D2)
    #   else
    #       ∇ ← the current direction of D1.

    def cross_boundary(self, d1, d2, a):
        gradient = self.calculate_gradient()
        # print(f"gradient: {gradient}")
        return self.turn(d1, d2, a, gradient)

    # Algorithm 3 Initially, robots are √λ apart; one inside and one outside
    def boundary_sketch(self, d1, d2):
        # D1, D2 ← the two robots
        # ∇ ← boundary gradient at point of crossing with line segment between D1 and D2
        # α ← √λ
        lambda_value = 0.1
        if self.inside(d1) ^ self.inside(d2):
            # D1 and D2 both move λ distance in the direction of ∇
            self.logger.info("Sandwich")
            return self.forward(d1, d2)

        if not self.inside(d1) and not self.inside(d2):
            self.logger.info("Outside")
            a = math.sqrt(lambda_value)
            return self.cross_boundary(d1, d2, a)
        elif self.inside(d1) and self.inside(d2):
            self.logger.info("Inside")
            a = -math.sqrt(lambda_value)
            return self.cross_boundary(d1, d2, a)

    def calculate_prev_direction(self):
        if self.target_position_vector.movement == PositionVector.FORWARD:
            direction = [self.target_position_vector.x, self.target_position_vector.y]
            # print(f"forward: {direction}")
            return direction
        if self.target_position_vector.movement == PositionVector.TURN:
            prev_vector = [self.target_position_vector.x, self.target_position_vector.y]
            angle =  self.target_position_vector.a * self.target_position_vector.p
            direction = rotate_vector(prev_vector, angle)
            # print(f"turn {self.target_position_vector.a} * {self.target_position_vector.p} = {angle * 57.2958}: {direction} {rotate_vector(prev_vector, angle)}")
            return direction

    def forward(self, d1, d2):

        gradient = self.calculate_gradient()

        position_vector = PositionVector()
        position_vector.movement = PositionVector.FORWARD
        position = average(d1, d2)
        position_vector.position = position

        if self.target_position_vector.movement == PositionVector.TURN:
            [position_vector.x, position_vector.y] = self.calculate_closest_gradient_tangent(self.calculate_prev_direction(), gradient)
        else:
            [position_vector.x, position_vector.y] = self.calculate_prev_direction()
        position_vector.position = position

        position_vector.distance = 10.0

        return position_vector

    def calculate_closest_gradient_tangent(self, direction, gradient):
        gradient_positive = rotate_vector(gradient, math.pi/2)
        gradient_negative = rotate_vector(gradient, -math.pi/2)

        angle_positive = calculate_angle(direction, gradient_positive)
        angle_negative = calculate_angle(direction, gradient_negative)


        # print(f"positive: {angle_positive} negative: {angle_negative}")
        if math.fabs(angle_positive) > math.fabs(angle_negative):
            perpendicular_gradient = gradient_negative
        else:
            perpendicular_gradient = gradient_positive

        # print(perpendicular_gradient)
        return perpendicular_gradient

    def turn(self, d1, d2, a, gradient):
        if a == self.target_position_vector.a:
            self.target_position_vector.p += 1
            # print(f"p : {self.target_position_vector.p}")
            return self.target_position_vector

        position_vector = PositionVector()
        position_vector.movement = PositionVector.TURN

        position = average(d1, d2)
        position_vector.position = position

        [position_vector.x, position_vector.y] = self.calculate_closest_gradient_tangent(self.calculate_prev_direction(), gradient)

        position_vector.position = position
        position_vector.a = a
        position_vector.p = 1

        # position_vector.gradient = gradient

        return position_vector

    def inside(self, d):
        return d.value > self.threshold

    def passed(self, average_position):
        if self.target_position_vector is None:
            return True

        if self.target_position_vector.movement == PositionVector.FORWARD:

            target_offset = difference_in_meters(average_position, self.target_position_vector.position)
            distance = np.dot(target_offset, [self.target_position_vector.x, self.target_position_vector.y])

            return distance > self.target_position_vector.distance
        else:
            center = calculate_turn_center(self.target_position_vector)
            target_offset = rotate_vector(difference_in_meters(self.target_position_vector.position, center), (self.target_position_vector.a * (self.target_position_vector.p - 1)))
            hyp = difference_in_meters(average_position, center)

            target_angle = math.fabs(self.target_position_vector.a)

            intermediate = ((target_offset[0] * hyp[0]) + (target_offset[1] * hyp[1])) / (magnitude(target_offset) * magnitude(hyp))
            if intermediate > 1:
                intermediate = 1
            angle = math.acos(intermediate)

            # print(f"Turn passed: {angle} > {target_angle} = {angle > target_angle}")
            return angle > target_angle


    def stop(self):
        self.navigate_subscription.dispose()
        self.leader_broadcast_subscription.dispose()
        self.receive_semaphore_subscription.dispose()
        self.sketch_control_subscription.dispose()