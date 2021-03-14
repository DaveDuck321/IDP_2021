class ReversingController:
    def __init__(self, drive_controller, tick_count):
        self.drive_controller = drive_controller
        self.tick_count = tick_count

    def __call__(self):
        if self.tick_count > 0:
            self.tick_count -= 1
            self.drive_controller.drive_backward()
            return False
        else:
            self.drive_controller.halt()
            return True
