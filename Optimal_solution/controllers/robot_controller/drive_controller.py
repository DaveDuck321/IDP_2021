import math

ERROR_TOLERANCE = 0.1  # Rad


class DriveController:
    def __init__(self, left_motor, right_motor):
        self._left_motor = left_motor
        self._right_motor = right_motor

        self._left_motor.setPosition(math.inf)
        self._right_motor.setPosition(math.inf)

        self._left_motor.setVelocity(0.0)
        self._right_motor.setVelocity(0.0)

    def halt(self):
        """
            Stop both motors immediately.
        """
        self._left_motor.setVelocity(0.0)
        self._right_motor.setVelocity(0.0)

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
        if error > ERROR_TOLERANCE:
            left_speed = max(max_speed - p * error, -max_speed)
            right_speed = max_speed
        elif -error > ERROR_TOLERANCE:
            left_speed = max_speed
            right_speed = max(max_speed + p * error, -max_speed)
        else:
            # Pointing at target, just drive
            left_speed = max_speed
            right_speed = max_speed

        if reverse:
            left_speed, right_speed = -right_speed, -left_speed

        self._left_motor.setPosition(math.inf)
        self._right_motor.setPosition(math.inf)

        # set drive motor speed
        self._left_motor.setVelocity(-left_speed)
        self._right_motor.setVelocity(-right_speed)

    def turn_toward_point(self, positioning_system, target, tolerance=math.pi / 180.0):
        """
            For the tick, rotate toward the bearing of a specified target point.
        """
        bearing_error = positioning_system.get_bearing_error(target)

        if abs(bearing_error) <= tolerance:
            return True

        p = 20.0
        max_speed = self._left_motor.getMaxVelocity()

        left_speed = min(max(-bearing_error * p, -max_speed), max_speed)
        right_speed = min(max(bearing_error * p, -max_speed), max_speed)

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
