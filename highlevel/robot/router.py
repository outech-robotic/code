"""
Protobuf message router module.
"""
from google.protobuf import json_format

from proto.gen.python.outech_pb2 import BusMessage
from highlevel.logger import LOGGER
from highlevel.robot.controller.match_action import MatchActionController
from highlevel.robot.controller.motion.localization import LocalizationController


class ProtobufRouter:
    """ Protobuf router decodes raw bytes messages and dispatches them to controllers. """
    def __init__(self, match_action_controller: MatchActionController,
                 localization_controller: LocalizationController):
        self.localization_controller = localization_controller
        self.match_action = match_action_controller

    async def translate_message(self, msg: bytes, source: str) -> None:
        """ Convert bytes to BusMessage. """
        bus_message = BusMessage()
        bus_message.ParseFromString(msg)
        printable_msg = json_format.MessageToJson(
            bus_message, including_default_value_fields=True)
        LOGGER.get().debug('msg_can', msg=printable_msg, source=source)
        await self.dispatch_message(bus_message, source)

    async def dispatch_message(self, bus_message: BusMessage,
                               source: str) -> None:
        """ Detect type of a BusMessage and call a controller. """
        type_msg = bus_message.WhichOneof("message_content")
        if type_msg == "heartbeat":
            pass
        elif type_msg == "movementEnded":
            self.localization_controller.movement_done(
                bus_message.movementEnded.blocked)
        elif type_msg == "encoderPosition":
            self.localization_controller.update_odometry_position(
                bus_message.encoderPosition.left_tick,
                bus_message.encoderPosition.right_tick)
        elif type_msg == "laserSensor":
            await self.match_action.set_laser_distances()
        elif type_msg == "pressureSensor":
            await self.match_action.set_pressures()
        elif type_msg == "debugLog":
            LOGGER.get().info("low_level_log",
                              content=bus_message.debugLog.content,
                              source=source)
        else:
            LOGGER.get().error("unhandled_protobuf_message",
                               message_type=type_msg,
                               source=source)
