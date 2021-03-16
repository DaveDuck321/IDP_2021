import math
from common import util

WALL_REJECTION_FRAC = 0.9
SEARCH_ANGLE = math.pi / 2.0   # angle of total search area, centered on initial direction
MAX_BLOCK_DIST = 0.4           # max distance away a block would be detected in m
BLOCK_IN_GRABBER_DIST = 0.15   # distance reading when block in grabber


class BlockSearch:
    FAILED = -1
    IN_PROGRESS = 0
    FOUND_BLOCK = 1
    TIMEOUT = 2

    def __init__(self, ir_sensor, positioning_system, drive_controller, pincer_controller):
        self.positioning_system = positioning_system
        self.drive_controller = drive_controller
        self.IR_sensor = ir_sensor
        self.pincer_controller = pincer_controller

        self.target_angle = (self.positioning_system.get_world_bearing() + (SEARCH_ANGLE / 2.0)) % (2 * math.pi)
        self.min_IR_dist = math.inf
        self.min_IR_angle = None
        self.block_found = False
        self.sweeping_back = False

        self.FUBAR_timer = 250

    def clean(self):
        self.drive_controller.halt()

    def __call__(self):
        IR_dist = self.IR_sensor.get_distance()

        wall_dist = WALL_REJECTION_FRAC * util.get_static_distance(
            self.positioning_system.get_2D_position(),
            self.positioning_system.get_world_bearing(),
            [], []
        )

        if not self.block_found:
            # Run the sweeping search using IR sensor to find direction of block
            if IR_dist < min(MAX_BLOCK_DIST, wall_dist):
                self.min_IR_angle = self.positioning_system.get_world_bearing()
                self.block_found = True
                self.sweeping_back = False

            if self.drive_controller.rotate_absolute_angle(self.positioning_system, self.target_angle):
                if not self.sweeping_back:
                    self.sweeping_back = True
                    self.target_angle = (self.target_angle - SEARCH_ANGLE + 2 * math.pi) % (2 * math.pi)
                else:
                    if IR_dist < min(MAX_BLOCK_DIST, wall_dist):
                        print("[Info]: Block Detected")
                        self.block_found = True
                        self.sweeping_back = False
                    else:
                        print("[Info]: No Block Found")
                        return self.FAILED
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
                    return self.TIMEOUT

                if IR_dist > BLOCK_IN_GRABBER_DIST:
                    self.drive_controller.drive_forward()
                else:
                    if self.pincer_controller.close_pincer():
                        return self.FOUND_BLOCK
        return False
