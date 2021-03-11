from common import protocol
import math
from common import util

BLOCK_OVERSHOOT = 0.05
BLOCK_IN_GRABBER_READING = 0.24


class BlockCollection:
    def __init__(self, robot_name, drive_controller, positioning_system,
                 pincer_controller, light, radio, IR_sensor):

        # Robot info
        self.robot_name = robot_name
        self.drive_controller = drive_controller
        self.positioning_system = positioning_system
        self.pincer_controller = pincer_controller
        self.light = light
        self.radio = radio
        self.IR_sensor = IR_sensor

        # Current navigation state
        self.__block_pos = None
        self.__block_color = None
        self.__starting_pos = None
        self.block_collected = False
        self.wait_for_pincer_index = 0
        self.target_bearing = None
        self.rolling_IR_Readings = []
        self.min_IR_dist = (float('inf'), 0.0)
        self.cur_step = self.drive_to_block

        # The controller should give the robot a new target now
        self.radio.send_message(protocol.AskForBlockPath(
            self.robot_name,
            self.positioning_system.get_2D_position(),
        ))

    def __call__(self):
        return self.cur_step()

    def set_block_pos(self, block_pos, block_color=None):
        self.__block_pos = block_pos
        self.__block_color = block_color

    def get_block_color(self):
        return self.__block_color

    def drive_to_block(self):
        if not self.drive_controller.has_waypoints():
            # print("waiting for waypoints")
            return False

        if self.drive_controller.navigate_waypoints(self.positioning_system):
            self.cur_step = self.face_block
        return False

    def face_block(self):
        if self.__block_pos is None:
            # print("waiting for block_pos")
            return False
        if self.drive_controller.turn_toward_point(self.positioning_system, self.__block_pos):
            self.__starting_pos = self.positioning_system.get_2D_position()
            self.cur_step = self.inspect_block
        return False

    def inspect_block(self):
        IR_dist = self.IR_sensor.get_distance()
        if IR_dist is not None and IR_dist < 0.2:
            self.drive_controller.set_waypoints([self.__block_pos])
            self.cur_step = self.drive_over_block
            print("drive over block")
        elif IR_dist is not None and IR_dist >= 0.2:
            self.target_bearing = (self.positioning_system.get_world_bearing() + math.pi / 6.0) % (2 * math.pi)
            self.min_IR_dist = (IR_dist, self.positioning_system.get_world_bearing())
            self.cur_step = self.IR_search_setup
            print("IR_search_setup")
        return False

    def IR_search_setup(self):
        if self.drive_controller.rotate_absolute_angle(self.positioning_system, self.target_bearing):
            print("[Info] Seting up IR search")

            self.cur_step = self.IR_search
            self.target_bearing = (self.positioning_system.get_world_bearing(
            ) - math.pi / 3.0 + 2 * math.pi) % (2 * math.pi)

            # Hackfix IR readings
            IR_dist = self.IR_sensor.get_distance()
            self.rolling_IR_Readings = [IR_dist] * 3
        return False

    def IR_search(self):
        self.rolling_IR_Readings.append(self.IR_sensor.get_distance())
        self.rolling_IR_Readings.pop(0)

        # Take rolling average
        IR_dist = sum(self.rolling_IR_Readings) / len(self.rolling_IR_Readings)

        if IR_dist is not None and IR_dist < self.min_IR_dist[0]:
            self.min_IR_dist = (IR_dist, self.positioning_system.get_world_bearing())

        if self.drive_controller.rotate_absolute_angle(self.positioning_system, self.target_bearing):
            robot_pos = self.positioning_system.get_2D_position()
            self.__block_pos = (
                robot_pos[0] + (self.min_IR_dist[0] - BLOCK_OVERSHOOT) * math.cos(self.min_IR_dist[1]),
                robot_pos[1] + (self.min_IR_dist[0] - BLOCK_OVERSHOOT) * math.sin(self.min_IR_dist[1])
            )
            self.drive_controller.set_waypoints([self.__block_pos])
            print(self.positioning_system.get_2D_position())
            print(self.__block_pos)
            print(self.min_IR_dist[0])
            self.cur_step = self.drive_over_block

    def drive_over_block(self):
        """
            Move the robot forward until the block is in the pincers.
        """
        if self.IR_sensor.get_distance() < BLOCK_IN_GRABBER_READING or \
                self.drive_controller.navigate_waypoints(self.positioning_system):

            self.pincer_controller.close_pincer()
            self.cur_step = lambda: self.wait_for_pincer(self.scan_block_color)

        return False

    def scan_block_color(self):
        self.drive_controller.halt()
        data = self.light.getValue()
        if data > 500:  # completely arbitrary threshold, ask Electronics for the real one
            print("Identified Green Block")
            if self.__block_color is not None and self.__block_color != "green":
                raise Exception("Detected green, was told it was red")
            self.__block_color = util.get_robot_name("green")
        else:
            print("Identified Red Block")
            if self.__block_color is not None and self.__block_color != "red":
                raise Exception("Detected red, was told it was green")
            self.__block_color = util.get_robot_name("red")

        if self.__block_color == self.robot_name:
            self.pincer_controller.close_pincer()
            self.block_collected = True
        else:
            self.pincer_controller.open_pincer()
            self.block_collected = False

        message = protocol.BlockScanResult(
            self.robot_name,
            self.positioning_system.get_2D_position(),
            self.__block_pos,
            self.__block_color,
            self.block_collected
        )
        self.radio.send_message(message)
        self.cur_step = lambda: self.wait_for_pincer(self.reverse_away)
        return False

    def wait_for_pincer(self, next_func):
        self.drive_controller.halt()
        self.wait_for_pincer_index += 1
        if self.wait_for_pincer_index == 50:
            self.wait_for_pincer_index = 0
            self.cur_step = next_func
        return False

    def reverse_away(self):
        self.drive_controller.set_waypoints([self.__starting_pos])
        if self.drive_controller.navigate_waypoints(self.positioning_system, reverse=True):
            if self.block_collected:
                self.cur_step = self.return_to_base
                return False
            else:
                return True

    def return_to_base(self):
        if not self.drive_controller.has_waypoints():
            print("waiting for waypoints")
            return False

        if self.drive_controller.navigate_waypoints(self.positioning_system):
            self.cur_step = self.face_dropoff_area
        return False

    def face_dropoff_area(self):
        if self.drive_controller.turn_toward_point(self.positioning_system, self.__block_pos):
            self.__starting_pos = self.positioning_system.get_2D_position()
            self.drive_controller.set_waypoints([self.__block_pos])
            self.cur_step = self.drive_to_dropoff
        return False

    def drive_to_dropoff(self):
        if self.drive_controller.navigate_waypoints(self.positioning_system):
            self.pincer_controller.open_pincer()
            self.block_collected = False
            self.drive_controller.set_waypoints([self.__starting_pos])
            self.cur_step = lambda: self.wait_for_pincer(self.reverse_away)
        return False
