"""
Protobuf message router module.
"""
from google.protobuf import json_format

from highlevel.logger import LOGGER
from highlevel.robot.controller.match_action import MatchActionController
from highlevel.robot.controller.motion.motion import MotionController
from highlevel.robot.controller.motion.position import PositionController
from idl.gen.python.BusMessage import BusMessage
from idl.gen.python.BusMessageUnion import BusMessageUnion
from idl.gen.python.DebugLog import DebugLog
from idl.gen.python.EncoderPositionMsg import EncoderPositionMsg
from proto.gen.python import outech_pb2


class FlatbuffersRouter:
    """ Flatbuffers router decodes raw bytes messages and dispatches them to controllers. """

    def __init__(self, match_action_controller: MatchActionController,
                 position_controller: PositionController,
                 motion_controller: MotionController):
        self.motion_controller = motion_controller
        self.position_controller = position_controller
        self.match_action = match_action_controller

    async def decode_message(self, msg: bytes, source: str) -> None:
        """ Convert bytes to BusMessage. """
        bus_message = BusMessage.GetRootAsBusMessage(msg, 0)
        LOGGER.get().debug('msg_can', source=source, msg=repr(msg))
        await self.dispatch_message(bus_message, source)

    async def dispatch_message(self, bus_message: BusMessage,
                               source: str) -> None:
        """ Detect type of a BusMessage and call a controller. """
        type_msg = bus_message.ContentType()
        content = bus_message.Content()
        if type_msg == BusMessageUnion.heartbeat:
            pass
        elif type_msg == BusMessageUnion.encoderPosition:
            msg = EncoderPositionMsg()
            msg.Init(content.Bytes, content.Pos)
            self.position_controller.update_odometry(
                msg.LeftTick(),
                msg.RightTick())
            self.motion_controller.trigger_update()

        elif type_msg == BusMessageUnion.laserSensor:
            await self.match_action.set_laser_distances()
        elif type_msg == BusMessageUnion.pressureSensor:
            await self.match_action.set_pressures()
        elif type_msg == BusMessageUnion.debugLog:
            msg = DebugLog()
            msg.Init(content.Bytes, content.Pos)

            LOGGER.get().info("low_level_log",
                              content=msg.Content(),
                              source=source)
        elif type_msg in [
            BusMessageUnion.moveWheelAtSpeed, BusMessageUnion.wheelPositionTarget,
            BusMessageUnion.pidConfig, BusMessageUnion.wheelTolerances, BusMessageUnion.wheelPWM,
            BusMessageUnion.wheelControlMode
        ]:
            # messages that are read back are ignored
            pass
        else:
            LOGGER.get().error("unhandled_flatbuffer_message",
                               message_type=type_msg,
                               source=source)


class ProtobufRouter:
    """ Protobuf router decodes raw bytes messages and dispatches them to controllers. """

    def __init__(self, match_action_controller: MatchActionController,
                 position_controller: PositionController,
                 motion_controller: MotionController):
        self.motion_controller = motion_controller
        self.position_controller = position_controller
        self.match_action = match_action_controller

    async def decode_message(self, msg: bytes, source: str) -> None:
        """ Convert bytes to BusMessage. """
        bus_message = outech_pb2.BusMessage()
        bus_message.ParseFromString(msg)
        printable_msg = json_format.MessageToJson(
            bus_message, including_default_value_fields=True)
        LOGGER.get().debug('msg_can', msg=printable_msg, source=source)
        await self.dispatch_message(bus_message, source)

    async def dispatch_message(self, bus_message: outech_pb2.BusMessage,
                               source: str) -> None:
        """ Detect type of a BusMessage and call a controller. """
        type_msg = bus_message.WhichOneof("message_content")
        if type_msg == "heartbeat":
            pass
        elif type_msg == "encoderPosition":
            self.position_controller.update_odometry(
                bus_message.encoderPosition.left_tick,
                bus_message.encoderPosition.right_tick)
            self.motion_controller.trigger_update()

        elif type_msg == "laserSensor":
            await self.match_action.set_laser_distances()
        elif type_msg == "pressureSensor":
            await self.match_action.set_pressures()
        elif type_msg == "debugLog":
            LOGGER.get().info("low_level_log",
                              content=bus_message.debugLog.content,
                              source=source)
        elif type_msg in [
            "moveWheelAtSpeed", "wheelPositionTarget", "pidConfig",
            "wheelTolerances", "wheelPWM", "wheelControlMode"
        ]:
            # messages that are read back are ignored
            pass
        else:
            LOGGER.get().error("unhandled_protobuf_message",
                               message_type=type_msg,
                               source=source)
