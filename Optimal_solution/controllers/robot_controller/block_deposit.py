from common import util


class DropoffController:
    def __init__(self, positioning_system, drive_controller, pincer_controller, robot_name):
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
