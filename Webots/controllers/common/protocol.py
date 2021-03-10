""" Defines the messages that may be sent over the radio """

from common import util


class Message:
    def __init__(self):
        self.type = self.get_type()

    def __repr__(self):
        return f"Type: {self.type}"

    @staticmethod
    def get_type():
        """
            This is a virtual method and should be overritten.
            Returns a JSON suitable string uniquely representing the type
        """
        raise NotImplementedError()

# ---------------------------------
# Robot --> Controller
# ---------------------------------


class BlockScanResult(Message):
    def __init__(self, robot_name, robot_position, block_position, is_green, is_moving_block):
        Message.__init__(self)
        self.robot_name = robot_name
        self.robot_position = util.filter_nans(robot_position)
        self.block_position = block_position
        self.is_green = is_green
        self.is_moving_block = is_moving_block

    @staticmethod
    def get_type():
        return "block_scan_result"

    @classmethod
    def build_from_JSON(cls, json_data):
        assert json_data["type"] == cls.get_type()

        return cls(
            json_data["robot_name"], json_data["robot_position"],
            json_data["block_position"], json_data["is_green"],
            json_data["is_moving_block"]
        )


class ScanDistanceReading(Message):
    def __init__(self, robot_name, robot_position, robot_bearing,
                 arm_angle, distance_readings):
        Message.__init__(self)
        self.robot_name = robot_name
        self.robot_position = util.filter_nans(robot_position)
        self.robot_bearing = util.filter_nan(robot_bearing)
        self.arm_angle = util.filter_nan(arm_angle)
        self.distance_readings = util.filter_nans(distance_readings)

    def __repr__(self):
        return f"Type:{self.type}, pos:{self.robot_position}, sensors:{self.distance_readings}"

    @staticmethod
    def get_type():
        return "scan_distance_reading"

    @classmethod
    def build_from_JSON(cls, json_data):
        assert json_data["type"] == cls.get_type()

        return cls(
            json_data["robot_name"],
            json_data["robot_position"], json_data["robot_bearing"],
            json_data["arm_angle"], json_data["distance_readings"]
        )

# ---------------------------------
# Controller --> Robot
# ---------------------------------


class WaypointList(Message):
    def __init__(self, waypoints):
        Message.__init__(self)
        self.waypoints = waypoints

    def __repr__(self):
        return f"Type:{self.type}, Waypoints:{self.waypoints}"

    @staticmethod
    def get_type():
        return "new_waypoint_list"

    @classmethod
    def build_from_JSON(cls, json_data):
        assert json_data["type"] == cls.get_type()

        return cls(json_data["waypoints"])


MESSAGE_MAPPINGS = {
    ScanDistanceReading.get_type(): ScanDistanceReading,
    WaypointList.get_type(): WaypointList,
    BlockScanResult.get_type(): BlockScanResult,
}


def build_message_from_JSON(json_data):
    assert json_data["type"] in MESSAGE_MAPPINGS

    message_type = MESSAGE_MAPPINGS[json_data["type"]]
    return message_type.build_from_JSON(json_data)
