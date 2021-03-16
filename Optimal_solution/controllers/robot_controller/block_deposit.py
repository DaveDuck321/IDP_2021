from common import util


DROPOFF_RADIUS = 0.17
DROPOFF_LOCATIONS = {"Small": (0.0, 0.4), "Fluffy": (0.0, -0.4)}


class BlockDeposit:
    def __init__(self, robot_name, positioning_system, drive_controller, pincer_controller):

        self.positioning_system = positioning_system
        self.drive_controller = drive_controller
        self.pincer_controller = pincer_controller

        self.dropoff_target = DROPOFF_LOCATIONS[robot_name]

    def __call__(self):
        current_position = self.positioning_system.get_2D_position()
        self.drive_controller.navigate_toward_point(self.positioning_system, self.dropoff_target)

        if util.get_distance(current_position, self.dropoff_target) < DROPOFF_RADIUS:
            self.drive_controller.halt()
            if self.pincer_controller.open_pincer():
                return True

        return False
