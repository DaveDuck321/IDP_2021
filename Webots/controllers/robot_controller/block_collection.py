

class BlockCollection:
    def __init__(self, drive_controller, positioning_system, pincer_controller, light, robot_color):
        self.drive_controller = drive_controller
        self.positioning_system = positioning_system
        self.pincer_controller = pincer_controller
        self.light = light
        self.robot_color = robot_color
        self.__block_pos = []
        self.__block_color = None
        self.__starting_pos = None
        self.block_collected = False
        self.cur_step = self.inspect_block

    def __call__(self):
        return self.cur_step()

    def set_block_pos(self, block_pos, block_color=None):
        self.__block_pos = block_pos
        self.__block_color = block_color

    def get_block_color(self):
        return self.__block_color

    def inspect_block(self):
        if self.drive_controller.turn_toward_point(self.positioning_system, self.__block_pos):
            self.__starting_pos = self.positioning_system.get_2D_position()
            self.drive_controller.set_waypoints([self.__block_pos])
            self.cur_step = self.drive_over_block
        return False

    def drive_over_block(self):
        if self.drive_controller.navigate_waypoints(self.positioning_system):
            self.cur_step = self.scan_block_color()
        return False

    def scan_block_color(self):
        data = self.light.getValue()
        print("[LOG] Color data: ", data)
        if data > 1000:  # completely arbitrary threshold, as Electronics for the real one
            if self.__block_color is not None and self.__block_color != "green":
                raise Exception("Detected green, was told it was red")
            self.__block_color = "green"
        else:
            if self.__block_color is not None and self.__block_color != "red":
                raise Exception("Detected red, was told it was green")
            self.__block_color = "red"

        if self.__block_color == self.robot_color:
            self.pincer_controller.close_pincer()
            self.block_collected = True

        self.cur_step = self.reverse_away
        self.cur_step = self.drive_controller.set_waypoints([self.__starting_pos])
        return False

    def reverse_away(self):
        if self.drive_controller.navigate_waypoints(self.positioning_system, reverse=True):
            if self.block_collected:
                self.cur_step = self.return_to_base()
                print("request route from controller back to base")
                # set the waypoints that were requested as the waypoints for drive_controller.navigate_waypoints()
                return False
            else:
                return True

    def return_to_base(self):
        if self.drive_controller.navigate_waypoints(self.positioning_system):
            self.__block_pos = (0.0, 0.0)  # set this as the point where the block needs to be left at
            self.cur_step = self.face_dropoff_area
        return False

    def face_dropoff_area(self):
        if self.drive_controller.turn_toward_point(self.positioning_system, self.__block_pos):
            self.__starting_pos = self.positioning_system.get_2D_position()
            self.drive_controller.set_waypoints([self.__block_pos])
            self.cur_step = self.drive_to_dropoff
            print("TODO: sort out return coordinates dropoff")
        return False

    def drive_to_dropoff(self):
        if self.drive_controller.navigate_waypoints(self.positioning_system):
            self.pincer_controller.open_pincer()
            self.block_collected = False
            self.cur_step = self.drive_controller.set_waypoints([self.__starting_pos])
            self.cur_step = self.reverse_away
        return False
