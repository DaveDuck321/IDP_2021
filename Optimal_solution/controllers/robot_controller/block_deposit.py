from common import util
import math

PINCER_OFFSET = 0.1


class BlockDeposit:
    def __init__(self, robot_name, positioning_system, drive_controller, pincer_controller):
        self.DROPOFF_RADIUS = 0.17

        self.positioning_system = positioning_system
        self.drive_controller = drive_controller
        self.pincer_controller = pincer_controller

        dropoff_locations = {"Small": (0.0, 0.4), "Fluffy": (0.0, -0.4)}
        self.dropoff_target = dropoff_locations[robot_name]
        self.drive_controller.set_waypoints([self.dropoff_target])

    def __call__(self):
        self.drive_controller.navigate_waypoints(self.positioning_system)
        if util.get_distance(self.positioning_system.get_2D_position(), self.dropoff_target) < self.DROPOFF_RADIUS:
            self.drive_controller.halt()
            if self.pincer_controller.open_pincer():
                return True
        return False

    def get_block_position(self):
        robot_pos = self.positioning_system.get_2D_position()
        robot_bearing = self.positioning_system.get_world_bearing()
        return (
            robot_pos[0] + PINCER_OFFSET * math.cos(robot_bearing),
            robot_pos[1] + PINCER_OFFSET * math.sin(robot_bearing)
        )
