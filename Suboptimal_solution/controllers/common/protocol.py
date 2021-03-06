""" Defines the messages that may be sent over the radio """

from common import util


class Message:
    def __init__(self, robot_name):
        self.type = self.get_type()
        self.robot_name = robot_name

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
    def __init__(self, robot_name, robot_position, block_position, color, is_moving_block):
        Message.__init__(self, robot_name)
        self.robot_position = util.filter_nans(robot_position)
        self.block_position = block_position
        self.color = color
        self.is_moving_block = is_moving_block

    @staticmethod
    def get_type():
        return "block_scan_result"

    @classmethod
    def build_from_JSON(cls, json_data):
        assert json_data["type"] == cls.get_type()

        return cls(
            json_data["robot_name"],
            json_data["robot_position"], json_data["block_position"],
            json_data["color"], json_data["is_moving_block"]
        )


class AskForBlockPath(Message):
    def __init__(self, robot_name, robot_position):
        Message.__init__(self, robot_name)
        self.robot_position = util.filter_nans(robot_position)

    @staticmethod
    def get_type():
        return "ask_for_block_path"

    @classmethod
    def build_from_JSON(cls, json_data):
        assert json_data["type"] == cls.get_type()

        return cls(
            json_data["robot_name"],
            json_data["robot_position"]
        )


class ReportBlockDropoff(Message):
    def __init__(self, robot_name, block_position):
        Message.__init__(self, robot_name)
        self.block_position = block_position

    @staticmethod
    def get_type():
        return "report_block_dropoff"

    @classmethod
    def build_from_JSON(cls, json_data):
        assert json_data["type"] == cls.get_type()

        return cls(
            json_data["robot_name"],
            json_data["block_position"]
        )


class ScanDistanceReading(Message):
    def __init__(self, robot_name, robot_position, robot_bearing,
                 arm_angle, distance_readings):
        Message.__init__(self, robot_name)
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


class RemoveWaypoints(Message):
    def __init__(self, robot_target):
        Message.__init__(self, robot_target)

    @staticmethod
    def get_type():
        return "remove_waypoints"

    @classmethod
    def build_from_JSON(cls, json_data):
        assert json_data["type"] == cls.get_type()

        return cls(json_data["robot_name"])


class WaypointList(Message):
    def __init__(self, robot_target, waypoints):
        Message.__init__(self, robot_target)
        self.waypoints = waypoints
        if len(waypoints) == 0:
            raise ValueError("Waypoint list should never be empty")

    def __repr__(self):
        return f"Type:{self.type}, Waypoints:{self.waypoints}"

    @staticmethod
    def get_type():
        return "new_waypoint_list"

    @classmethod
    def build_from_JSON(cls, json_data):
        assert json_data["type"] == cls.get_type()

        return cls(json_data["robot_name"], json_data["waypoints"])


class KillImmediately(Message):
    def __init__(self, robot_name):
        Message.__init__(self, robot_name)

    @staticmethod
    def get_type():
        return "kill_immediately"

    @classmethod
    def build_from_JSON(cls, json_data):
        assert json_data["type"] == cls.get_type()

        return cls(json_data["robot_name"])


MESSAGE_MAPPINGS = {
    ScanDistanceReading.get_type(): ScanDistanceReading,
    WaypointList.get_type(): WaypointList,
    BlockScanResult.get_type(): BlockScanResult,
    AskForBlockPath.get_type(): AskForBlockPath,
    ReportBlockDropoff.get_type(): ReportBlockDropoff,
    KillImmediately.get_type(): KillImmediately,
    RemoveWaypoints.get_type(): RemoveWaypoints,
}


def build_message_from_JSON(json_data):
    assert json_data["type"] in MESSAGE_MAPPINGS

    message_type = MESSAGE_MAPPINGS[json_data["type"]]
    return message_type.build_from_JSON(json_data)
