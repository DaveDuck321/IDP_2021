""" Defines the messages that may be sent over the radio """


class Message:
    def __init__(self):
        self.type = self.get_type()

    @staticmethod
    def get_type():
        """
            This is a virtual method and should be overritten.
            Returns a JSON suitable string uniquely representing the type
        """
        raise NotImplementedError()


class ScanReading(Message):
    def __init__(self, readings):
        Message.__init__(self)

    @staticmethod
    def get_type():
        return "1"

    @classmethod
    def build_from_JSON(cls, json_data):
        assert json_data["type"] == cls.get_type()
        return cls(json_data["readings"])


MESSAGE_MAPPINGS = {ScanReading.get_type(): ScanReading}


def build_message_from_JSON(json_data):
    assert json_data["type"] in MESSAGE_MAPPINGS

    message_type = MESSAGE_MAPPINGS[json_data["type"]]
    return message_type.build_from_JSON(json_data)
