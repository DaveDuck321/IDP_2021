from common import util

import math


class DriveController:
    def __init__(self, left_motor, right_motor):
        self._left_motor = left_motor
        self._right_motor = right_motor

        self._left_motor.setPosition(math.inf)
        self._right_motor.setPosition(math.inf)

        self._left_motor.setVelocity(0.0)
        self._right_motor.setVelocity(0.0)

        self.__waypoints = []
        self.__waypoint_index = 0

    def set_waypoints(self, waypoints):
        """
            The controller will store the list of relevent waypoint tuples.
            It will attempt to navigate towards the next one in the sequence
            when navigate_along_waypoints is called.
        """
        self.__waypoint_index = 0
        self.__waypoints = waypoints

    def has_waypoints(self):
        return len(self.__waypoints) > 0

    def halt(self):
        """
            Stop both motors immediately.
        """
        self._left_motor.setVelocity(0.0)
        self._right_motor.setVelocity(0.0)

    def navigate_waypoints(self, positioning_system, tolerance=0.03, reverse=False):
        """
            For this tick, attempt to navigate towards the next waypoint.
            Returns True if the final waypoint has been reached.
        """
        if self.__waypoint_index >= len(self.__waypoints):
            self.__waypoints = []
            self.__waypoint_index = 0
            return True

        target_waypoint = self.__waypoints[self.__waypoint_index]
        current_position = positioning_system.get_2D_position()

        self.navigate_toward_point(positioning_system, target_waypoint, reverse)

        if util.get_distance(current_position, target_waypoint) < tolerance:
            self.__waypoint_index += 1

        if self.__waypoint_index == len(self.__waypoints):
            self.__waypoints = []
            self.__waypoint_index = 0
            return True
        return False

    def navigate_toward_point(self, positioning_system, destination, reverse=False):
        max_speed = self._left_motor.getMaxVelocity()

        # the importance of this value depends on the distance away from
        # the point. Some fine tuning later down the line may be necessary
        p = 20.0
        error = positioning_system.get_bearing_error(destination, reverse)

        """
        left_speed = min(max(max_speed / 2 - error * p, -max_speed), max_speed)
        right_speed = min(max(max_speed / 2 + error * p, -max_speed), max_speed)

        """
        if error > 0.0:
            left_speed = max(max_speed - p * error, -max_speed)
            right_speed = max_speed
        elif error < 0.0:
            left_speed = max_speed
            right_speed = max(max_speed + p * error, -max_speed)

        if reverse:
            left_speed, right_speed = -right_speed, -left_speed

        # set drive motor speed
        self._left_motor.setVelocity(-left_speed)
        self._right_motor.setVelocity(-right_speed)

    def turn_toward_point(self, positioning_system, target, tolerance=math.pi / 180.0):
        """
            For the tick, rotate toward the bearing of a specified target point.
        """
        error = positioning_system.get_bearing_error(target)
        if abs(error) <= tolerance:
            return True

        p = 20.0
        max_speed = self._left_motor.getMaxVelocity() / 2

        left_speed = min(max(-error * p, -max_speed), max_speed)
        right_speed = min(max(error * p, -max_speed), max_speed)

        self._left_motor.setVelocity(-left_speed)
        self._right_motor.setVelocity(-right_speed)
        return False

    def rotate_absolute_angle(self, positioning_system, goal_bearing, tolerance=math.pi / 180.0):
        current_bearing = positioning_system.get_world_bearing()
        bearing_error = current_bearing - goal_bearing
        if bearing_error > math.pi:
            bearing_error -= 2 * math.pi
        elif bearing_error < -math.pi:
            bearing_error += 2 * math.pi

        if abs(bearing_error) <= tolerance:
            return True

        p = 20.0
        max_speed = self._left_motor.getMaxVelocity()

        left_speed = min(max(-bearing_error * p, -max_speed), max_speed)
        right_speed = min(max(bearing_error * p, -max_speed), max_speed)

        self._left_motor.setVelocity(-left_speed)
        self._right_motor.setVelocity(-right_speed)
        return False

    def drive_forward(self):
        max_speed = self._left_motor.getMaxVelocity()
        self._left_motor.setVelocity(-max_speed)
        self._right_motor.setVelocity(-max_speed)

    def drive_backward(self):
        max_speed = self._left_motor.getMaxVelocity()
        self._left_motor.setVelocity(max_speed)
        self._right_motor.setVelocity(max_speed)
