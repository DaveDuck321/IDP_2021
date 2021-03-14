import math


class BlockSearch:
    def __init__(self, IR_sensor, positioning_system, drive_controller, pincer_controller):
        self.SEARCH_ANGLE = math.pi / 3.0   # angle of total search area, centered on initial direction
        self.MAX_BLOCK_DIST = 0.4           # max distance away a block would be detected in m
        self.BLOCK_IN_GRABBER_DIST = 0.15   # distance reading when block in grabber

        self.positioning_system = positioning_system
        self.drive_controller = drive_controller
        self.IR_sensor = IR_sensor
        self.pincer_controller = pincer_controller

        self.target_angle = (self.positioning_system.get_world_bearing() + (self.SEARCH_ANGLE / 2.0)) % (2 * math.pi)
        self.min_IR_dist = math.inf
        self.min_IR_angle = None
        self.block_found = False
        self.sweeping_back = False
        self.rolling_IR_readings = [self.IR_sensor.get_distance()] * 3

        self.FUBAR_timer = 250

    def __call__(self):
        self.rolling_IR_readings.append(self.IR_sensor.get_distance())
        self.rolling_IR_readings.pop(0)

        IR_dist = sum(self.rolling_IR_readings) / len(self.rolling_IR_readings)

        if not self.block_found:
            # Run the sweeping search using IR sensor to find direction of block
            if IR_dist < self.MAX_BLOCK_DIST:
                self.min_IR_angle = self.positioning_system.get_world_bearing()
                self.block_found = True
                self.sweeping_back = False

            if self.drive_controller.rotate_absolute_angle(self.positioning_system, self.target_angle):
                if not self.sweeping_back:
                    self.sweeping_back = True
                    self.target_angle = (self.target_angle - self.SEARCH_ANGLE + 2 * math.pi) % (2 * math.pi)
                else:
                    if self.min_IR_dist < self.MAX_BLOCK_DIST:
                        print("[Info]: Block Detected")
                        self.block_found = True
                        self.sweeping_back = False
                    else:
                        print("[Info]: No Block Found")
                        return True
        else:
            # If a block has been found, rotate towards it, then drive until it is within
            # the gripper, then close the gripper
            if not self.sweeping_back:
                # Make sure the robot is facing the right way before driving forward
                if self.drive_controller.rotate_absolute_angle(self.positioning_system, self.min_IR_angle):
                    self.sweeping_back = True
            else:
                self.FUBAR_timer -= 1
                if self.FUBAR_timer == 0:
                    print('[Warning]: Took too long to reach block')
                    self.drive_controller.halt()
                    self.pincer_controller.close_pincer()
                    return True
                if IR_dist > self.BLOCK_IN_GRABBER_DIST:
                    self.drive_controller.drive_forward()
                else:
                    self.drive_controller.halt()
                    if self.pincer_controller.close_pincer():
                        return True
        return False
