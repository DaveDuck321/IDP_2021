import math


class PincerController:
    def __init__(self, left_motor, right_motor):
        self.PINCER_TIMER = 25

        self.left_motor = left_motor
        self.right_motor = right_motor

        # 2 motors for Webots simulation. In reality, the pincer arms are
        # attached via a gearing system so their velocities are fixed relative
        # to eachother. left_motor.getMaxVelocity() is set by the mechanics.
        max_servo_velocity = self.left_motor.getMaxVelocity()
        self.left_motor.setPosition(0.0)
        self.right_motor.setPosition(0.0)

        self.left_motor.setVelocity(max_servo_velocity)
        self.right_motor.setVelocity(max_servo_velocity)

        self.is_closed = False
        self._timer = 0

    def open_pincer(self):
        self._timer += 1
        open_angle = math.pi / 5.0
        self.left_motor.setPosition(open_angle)
        self.right_motor.setPosition(-open_angle)
        if self._timer > self.PINCER_TIMER:
            # print("[INFO]: Pincer Opened")
            self._timer = 0
            self.is_closed = False
            return True
        return False

    def close_pincer(self):
        self._timer += 1
        self.left_motor.setPosition(0.0)
        self.right_motor.setPosition(0.0)
        if self._timer > self.PINCER_TIMER:
            # print("[INFO]: Pincer Closed")
            self._timer = 0
            self.is_closed = True
            return True
        return False
