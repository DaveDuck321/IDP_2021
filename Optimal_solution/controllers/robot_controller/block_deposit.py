from common import util

DROPOFF_ANGLE_TOLERANCE = 5 / 360 * 3  # Tolerance is 5 degrees
DROPOFF_RADIUS = 0.17
DROPOFF_LOCATIONS = {"Small": (0.0, 0.4), "Fluffy": (0.0, -0.4)}


class BlockDeposit:
    def __init__(self, robot_name, positioning_system, drive_controller, pincer_controller):

        self.positioning_system = positioning_system
        self.drive_controller = drive_controller
        self.pincer_controller = pincer_controller

        self.dropoff_target = DROPOFF_LOCATIONS[robot_name]
        self.opening_pincer = False

    def __call__(self):
        if self.opening_pincer:
            return self.pincer_controller.open_pincer()

        current_position = self.positioning_system.get_2D_position()

        if util.get_distance(current_position, self.dropoff_target) < DROPOFF_RADIUS:
            is_aligned = self.drive_controller.turn_toward_point(
                self.positioning_system,
                self.dropoff_target,
                DROPOFF_ANGLE_TOLERANCE
            )
            if is_aligned:
                self.drive_controller.halt()
                self.opening_pincer = True

        else:
            self.drive_controller.navigate_toward_point(self.positioning_system, self.dropoff_target)

        return False
