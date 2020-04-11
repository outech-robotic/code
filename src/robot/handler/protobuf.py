"""
Protobuf message handler module.
"""
from proto.gen.pb_pb2 import BusMessage
from src.logger import LOGGER
from src.robot.controller.match_action import MatchActionController


class ProtobufHandler:
    """ Protobuf handler : take bytes and decode to BusMessage """

    def __init__(self, match_action_controller: MatchActionController):
        self.match_action = match_action_controller

    async def translate_message(self, msg: bytes) -> None:
        """ Convert bytes to BusMessage. """
        bus_message = BusMessage()
        bus_message.ParseFromString(msg)
        LOGGER.get().debug('msg_can', msg=bus_message)
        await self.dispatch_message(bus_message)

    async def dispatch_message(self, bus_message: BusMessage) -> None:
        """ Detect type of a BusMessage and call a controller. """
        type_msg = bus_message.WhichOneof("message_content")
        if type_msg == "heartbeat":
            pass
        elif type_msg == "encoderPosition":
            if bus_message.encoderPosition.right_tick and bus_message.encoderPosition.left_tick:
                pass
        elif type_msg == "laserSensor":
            await self.match_action.set_laser_distances()
        elif type_msg == "pressureSensor":
            await self.match_action.set_pressures()
        else:
            LOGGER.get().error("wrong_protobuf_field")
