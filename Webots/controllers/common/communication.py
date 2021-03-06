""" Tools to enable communication via the emitter and receiver """

import json
from common.protocol import Message, build_message_from_JSON


class Radio:
    def __init__(self, emitter, receiver, sampling_rate=5):
        self._emitter = emitter
        self._receiver = receiver
        self._receiver.enable(sampling_rate)

    def set_receiver_channel(self, channel):
        """ Sets the receiver channel """
        self._receiver.setChannel(channel)

    def set_emitter_channel(self, channel):
        """ Sets the emitter channel """
        self._emitter.setChannel(channel)

    def is_queue_empty(self):
        """
            Returns a bool, True if the receiver queue is currently empty.
        """
        return self._receiver.getQueueLength() == 0

    def get_messages(self):
        """
            Returns an iterator of received Message objects.
        """
        while not self.is_queue_empty():
            # Parse next message into common format
            received_str = self._receiver.getData()
            json_result = json.loads(received_str)

            # Convert this to a class and return the result
            yield build_message_from_JSON(json_result)

            # Get ready to receive the next packet
            self._receiver.nextPacket()

    def broadcast_message(self, message_obj):
        """
            Broadcasts a single message across all channels.
        """
        old_channel = self._emitter.getChannel()

        self._emitter.setChannel(self._emitter.CHANNEL_BROADCAST)
        self.send_message(message_obj)

        # Restore settings for normal operation
        self._emitter.setChannel(old_channel)

    def send_message(self, message_obj, debug=False):
        """
            Sends a single Message object via the emitter.
        """
        assert isinstance(message_obj, Message)

        if debug:
            print("[INFO] Sent message:", message_obj)

        message_string = json.dumps(message_obj, default=vars)
        self._emitter.send(message_string.encode('utf-8'))
