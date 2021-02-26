""" Tools to enable communication via the emitter and receiver """

import json
from protocol import Message, build_message_from_JSON


class Radio:
    def __init__(self, emitter, receiver):
        self.emitter = emitter
        self.receiver = receiver

    def is_queue_empty(self):
        """
            Returns a bool, True if the receiver queue is currently empty.
        """
        return self.receiver.getQueueLength() == 0

    def get_messages(self):
        """
            Returns an iterator of received Message objects.
        """
        while not self.is_queue_empty():
            # Parse next message into common format
            received_str = self.receiver.getData()
            json_result = json.loads(received_str)

            # Convert this to a class and return the result
            yield build_message_from_JSON(json_result)

            # Get ready to receive the next packet
            self.receiver.nextPacket()

    def send_message(self, messages):
        """
            Sends a number of Message objects via the emitter.
        """
        for message_obj in messages:
            assert isinstance(message_obj, Message)

            message_string = json.dumps(message_obj)
            self.emitter.send(message_string)
