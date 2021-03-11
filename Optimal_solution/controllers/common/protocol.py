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
                 arm_angle, distance_readings, holding_block):
        Message.__init__(self, robot_name)
        self.robot_position = robot_position
        self.robot_bearing = robot_bearing
        self.arm_angle = arm_angle
        self.distance_readings = distance_readings
        self.holding_block = holding_block

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
            json_data["arm_angle"], json_data["distance_readings"],
            json_data["holding_block"]
        )

# ---------------------------------
# Controller --> Robot
# ---------------------------------


class GiveRobotTarget(Message):
    def __init__(self, robot, target):
        Message.__init__(self, robot)
        self.target = target

    @staticmethod
    def get_type():
        return "give_robot_target"

    @classmethod
    def build_from_JSON(cls, json_data):
        assert json_data["type"] == cls.get_type()

        return cls(json_data["robot_name"], json_data["target"])


class AskRobotTakeover(Message):
    def __init__(self, robot, target):
        Message.__init__(self, robot)
        self.target = target

    @staticmethod
    def get_type():
        return "ask_robot_takeover"

    @classmethod
    def build_from_JSON(cls, json_data):
        assert json_data["type"] == cls.get_type()

        return cls(json_data["robot_name"], json_data["target"])


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
    # Robot
    ScanDistanceReading.get_type(): ScanDistanceReading,
    BlockScanResult.get_type(): BlockScanResult,
    ReportBlockDropoff.get_type(): ReportBlockDropoff,

    # Controller
    GiveRobotTarget.get_type(): GiveRobotTarget,
    AskRobotTakeover.get_type(): AskRobotTakeover,
    KillImmediately.get_type(): KillImmediately,
}


def build_message_from_JSON(json_data):
    assert json_data["type"] in MESSAGE_MAPPINGS

    message_type = MESSAGE_MAPPINGS[json_data["type"]]
    return message_type.build_from_JSON(json_data)
