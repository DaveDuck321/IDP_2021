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

    def halt(self):
        """
            Stop both motors immediately.
        """
        self._left_motor.setVelocity(0.0)
        self._right_motor.setVelocity(0.0)

    def navigate_waypoints(self, positioning_system, tolerance=0.03):
        """
            For this tick, attempt to navigate towards the next waypoint.
            Returns True if the final waypoint has been reached.
        """
        if self.__waypoint_index >= len(self.__waypoints):
            return True

        target_waypoint = self.__waypoints[self.__waypoint_index]
        current_position = positioning_system.get_2D_position()

        self.navigate_toward_point(positioning_system, target_waypoint)

        if util.get_distance(current_position, target_waypoint) < tolerance:
            self.__waypoint_index += 1

        return self.__waypoint_index == len(self.__waypoints)

    def navigate_toward_point(self, positioning_system, destination):
        max_speed = self._left_motor.getMaxVelocity()

        # the importance of this value depends on the distance away from
        # the point. Some fine tuning later down the line may be necessary
        p = 10.0

        current_position = positioning_system.get_2D_position()
        current_bearing = positioning_system.get_world_bearing()

        # Use proportional control based on error in orientation between
        # the current robot orientation and the bearing of the waypoint
        goal_bearing = util.get_bearing(destination, current_position)

        # Limit bearing to range -pi/2 to pi/2
        if current_bearing < -math.pi / 2.0 or current_bearing > math.pi / 2.0:
            current_bearing = positioning_system.get_world_bearing(-1)
            goal_bearing = util.get_bearing(current_position, destination)

        error = current_bearing - goal_bearing

        left_speed = min(max(max_speed / 2 - error * p, -max_speed), max_speed)
        right_speed = min(max(max_speed / 2 + error * p, -max_speed), max_speed)

        """
        if error > 0.0:
            left_speed = max(max_speed - p * error, -max_speed)
            right_speed = max_speed
        elif error < 0.0:
            left_speed = max_speed
            right_speed = max(max_speed + p * error, -max_speed)
        """

        # set drive motor speed
        self._left_motor.setVelocity(-left_speed)
        self._right_motor.setVelocity(-right_speed)
