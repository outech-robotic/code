"""
Protobuf message router module.
"""
from google.protobuf import json_format

from highlevel.logger import LOGGER
from highlevel.robot.controller.match_action import MatchActionController
from highlevel.robot.controller.sensor import SensorController
from highlevel.robot.controller.motion.motion import MotionController
from highlevel.robot.controller.motion.position import PositionController
from proto.gen.python.outech_pb2 import BusMessage


class ProtobufRouter:
    """ Protobuf router decodes raw bytes messages and dispatches them to controllers. """
    def __init__(self, match_action_controller: MatchActionController,
                 position_controller: PositionController,
                 motion_controller: MotionController,
                 sensor_controller: SensorController):
        self.sensor_controller = sensor_controller
        self.motion_controller = motion_controller
        self.position_controller = position_controller
        self.match_action = match_action_controller

    async def decode_message(self, msg: bytes, source: str) -> None:
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
        elif type_msg == "encoderPosition":
            self.position_controller.update_odometry(
                bus_message.encoderPosition.left_tick,
                bus_message.encoderPosition.right_tick)
            self.motion_controller.trigger_update()

        elif type_msg == "laserSensor":
            data = [
                bus_message.laserSensor.distance_back_left,
                bus_message.laserSensor.distance_front_left,
                bus_message.laserSensor.distance_front_right,
                bus_message.laserSensor.distance_back_right,
            ]
            self.sensor_controller.update_laser_sensors(data)
        elif type_msg == "pressureSensor":
            states = [
                bus_message.pressureSensor.on_left,
                bus_message.pressureSensor.on_center_left,
                bus_message.pressureSensor.on_center,
                bus_message.pressureSensor.on_center_right,
                bus_message.pressureSensor.on_right,
            ]
            self.sensor_controller.update_pressure_sensors(states)
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
