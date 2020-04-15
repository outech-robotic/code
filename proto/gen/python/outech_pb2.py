# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: outech.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='outech.proto',
  package='src',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=b'\n\x0coutech.proto\x12\x03src\"\x0e\n\x0cHeartbeatMsg\"\x0f\n\rStopMovingMsg\"#\n\x10MovementEndedMsg\x12\x0f\n\x07\x62locked\x18\x01 \x01(\x08\";\n\x12\x45ncoderPositionMsg\x12\x11\n\tleft_tick\x18\x01 \x01(\x11\x12\x12\n\nright_tick\x18\x02 \x01(\x11\"t\n\x0eMotionLimitMsg\x12\x19\n\x11translation_speed\x18\x02 \x01(\r\x12\x16\n\x0erotation_speed\x18\x03 \x01(\r\x12\x13\n\x0bwheel_speed\x18\x04 \x01(\r\x12\x1a\n\x12wheel_acceleration\x18\x05 \x01(\r\"B\n\x0cPIDConfigMsg\x12\x0e\n\x06pid_id\x18\x01 \x01(\r\x12\n\n\x02kp\x18\x06 \x01(\r\x12\n\n\x02ki\x18\x07 \x01(\r\x12\n\n\x02kd\x18\x08 \x01(\r\"O\n\x17SetMotionControlModeMsg\x12\r\n\x05speed\x18\x01 \x01(\x08\x12\x13\n\x0btranslation\x18\x02 \x01(\x08\x12\x10\n\x08rotation\x18\x03 \x01(\x08\"L\n\x13MoveWheelAtSpeedMsg\x12\x19\n\x11left_tick_per_sec\x18\x01 \x01(\x11\x12\x1a\n\x12right_tick_per_sec\x18\x02 \x01(\x11\"\x1d\n\x0cTranslateMsg\x12\r\n\x05ticks\x18\x01 \x01(\x11\"\x1a\n\tRotateMsg\x12\r\n\x05ticks\x18\x01 \x01(\x11\"%\n\x08ServoMsg\x12\n\n\x02id\x18\x01 \x01(\r\x12\r\n\x05\x61ngle\x18\x02 \x01(\x11\")\n\x0fPumpAndValveMsg\x12\n\n\x02id\x18\x01 \x01(\r\x12\n\n\x02on\x18\x02 \x01(\x08\"\x84\x01\n\x0eLaserSensorMsg\x12\x1b\n\x13\x64istance_front_left\x18\x01 \x01(\r\x12\x1c\n\x14\x64istance_front_right\x18\x02 \x01(\r\x12\x1a\n\x12\x64istance_back_left\x18\x03 \x01(\r\x12\x1b\n\x13\x64istance_back_right\x18\x04 \x01(\r\"z\n\x11PressureSensorMsg\x12\x0f\n\x07on_left\x18\x01 \x01(\x08\x12\x16\n\x0eon_center_left\x18\x02 \x01(\x08\x12\x11\n\ton_center\x18\x03 \x01(\x08\x12\x17\n\x0fon_center_right\x18\x04 \x01(\x08\x12\x10\n\x08on_right\x18\x05 \x01(\x08\"\x93\x05\n\nBusMessage\x12&\n\theartbeat\x18\x01 \x01(\x0b\x32\x11.src.HeartbeatMsgH\x00\x12(\n\nstopMoving\x18\x02 \x01(\x0b\x32\x12.src.StopMovingMsgH\x00\x12.\n\rmovementEnded\x18\x03 \x01(\x0b\x32\x15.src.MovementEndedMsgH\x00\x12\x32\n\x0f\x65ncoderPosition\x18\x04 \x01(\x0b\x32\x17.src.EncoderPositionMsgH\x00\x12&\n\tpidConfig\x18\x05 \x01(\x0b\x32\x11.src.PIDConfigMsgH\x00\x12*\n\x0bmotionLimit\x18\x06 \x01(\x0b\x32\x13.src.MotionLimitMsgH\x00\x12<\n\x14setMotionControlMode\x18\x07 \x01(\x0b\x32\x1c.src.SetMotionControlModeMsgH\x00\x12\x34\n\x10moveWheelAtSpeed\x18\x08 \x01(\x0b\x32\x18.src.MoveWheelAtSpeedMsgH\x00\x12&\n\ttranslate\x18\t \x01(\x0b\x32\x11.src.TranslateMsgH\x00\x12 \n\x06rotate\x18\n \x01(\x0b\x32\x0e.src.RotateMsgH\x00\x12\x1e\n\x05servo\x18\x0b \x01(\x0b\x32\r.src.ServoMsgH\x00\x12,\n\x0cpumpAndValve\x18\x0c \x01(\x0b\x32\x14.src.PumpAndValveMsgH\x00\x12*\n\x0blaserSensor\x18\r \x01(\x0b\x32\x13.src.LaserSensorMsgH\x00\x12\x30\n\x0epressureSensor\x18\x0e \x01(\x0b\x32\x16.src.PressureSensorMsgH\x00\x42\x11\n\x0fmessage_contentb\x06proto3'
)




_HEARTBEATMSG = _descriptor.Descriptor(
  name='HeartbeatMsg',
  full_name='src.HeartbeatMsg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=21,
  serialized_end=35,
)


_STOPMOVINGMSG = _descriptor.Descriptor(
  name='StopMovingMsg',
  full_name='src.StopMovingMsg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=37,
  serialized_end=52,
)


_MOVEMENTENDEDMSG = _descriptor.Descriptor(
  name='MovementEndedMsg',
  full_name='src.MovementEndedMsg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='blocked', full_name='src.MovementEndedMsg.blocked', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=54,
  serialized_end=89,
)


_ENCODERPOSITIONMSG = _descriptor.Descriptor(
  name='EncoderPositionMsg',
  full_name='src.EncoderPositionMsg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='left_tick', full_name='src.EncoderPositionMsg.left_tick', index=0,
      number=1, type=17, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='right_tick', full_name='src.EncoderPositionMsg.right_tick', index=1,
      number=2, type=17, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=91,
  serialized_end=150,
)


_MOTIONLIMITMSG = _descriptor.Descriptor(
  name='MotionLimitMsg',
  full_name='src.MotionLimitMsg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='translation_speed', full_name='src.MotionLimitMsg.translation_speed', index=0,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rotation_speed', full_name='src.MotionLimitMsg.rotation_speed', index=1,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='wheel_speed', full_name='src.MotionLimitMsg.wheel_speed', index=2,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='wheel_acceleration', full_name='src.MotionLimitMsg.wheel_acceleration', index=3,
      number=5, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=152,
  serialized_end=268,
)


_PIDCONFIGMSG = _descriptor.Descriptor(
  name='PIDConfigMsg',
  full_name='src.PIDConfigMsg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='pid_id', full_name='src.PIDConfigMsg.pid_id', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='kp', full_name='src.PIDConfigMsg.kp', index=1,
      number=6, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='ki', full_name='src.PIDConfigMsg.ki', index=2,
      number=7, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='kd', full_name='src.PIDConfigMsg.kd', index=3,
      number=8, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=270,
  serialized_end=336,
)


_SETMOTIONCONTROLMODEMSG = _descriptor.Descriptor(
  name='SetMotionControlModeMsg',
  full_name='src.SetMotionControlModeMsg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='speed', full_name='src.SetMotionControlModeMsg.speed', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='translation', full_name='src.SetMotionControlModeMsg.translation', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rotation', full_name='src.SetMotionControlModeMsg.rotation', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=338,
  serialized_end=417,
)


_MOVEWHEELATSPEEDMSG = _descriptor.Descriptor(
  name='MoveWheelAtSpeedMsg',
  full_name='src.MoveWheelAtSpeedMsg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='left_tick_per_sec', full_name='src.MoveWheelAtSpeedMsg.left_tick_per_sec', index=0,
      number=1, type=17, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='right_tick_per_sec', full_name='src.MoveWheelAtSpeedMsg.right_tick_per_sec', index=1,
      number=2, type=17, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=419,
  serialized_end=495,
)


_TRANSLATEMSG = _descriptor.Descriptor(
  name='TranslateMsg',
  full_name='src.TranslateMsg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ticks', full_name='src.TranslateMsg.ticks', index=0,
      number=1, type=17, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=497,
  serialized_end=526,
)


_ROTATEMSG = _descriptor.Descriptor(
  name='RotateMsg',
  full_name='src.RotateMsg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ticks', full_name='src.RotateMsg.ticks', index=0,
      number=1, type=17, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=528,
  serialized_end=554,
)


_SERVOMSG = _descriptor.Descriptor(
  name='ServoMsg',
  full_name='src.ServoMsg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='src.ServoMsg.id', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='angle', full_name='src.ServoMsg.angle', index=1,
      number=2, type=17, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=556,
  serialized_end=593,
)


_PUMPANDVALVEMSG = _descriptor.Descriptor(
  name='PumpAndValveMsg',
  full_name='src.PumpAndValveMsg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='src.PumpAndValveMsg.id', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='on', full_name='src.PumpAndValveMsg.on', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=595,
  serialized_end=636,
)


_LASERSENSORMSG = _descriptor.Descriptor(
  name='LaserSensorMsg',
  full_name='src.LaserSensorMsg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='distance_front_left', full_name='src.LaserSensorMsg.distance_front_left', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='distance_front_right', full_name='src.LaserSensorMsg.distance_front_right', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='distance_back_left', full_name='src.LaserSensorMsg.distance_back_left', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='distance_back_right', full_name='src.LaserSensorMsg.distance_back_right', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=639,
  serialized_end=771,
)


_PRESSURESENSORMSG = _descriptor.Descriptor(
  name='PressureSensorMsg',
  full_name='src.PressureSensorMsg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='on_left', full_name='src.PressureSensorMsg.on_left', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='on_center_left', full_name='src.PressureSensorMsg.on_center_left', index=1,
      number=2, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='on_center', full_name='src.PressureSensorMsg.on_center', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='on_center_right', full_name='src.PressureSensorMsg.on_center_right', index=3,
      number=4, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='on_right', full_name='src.PressureSensorMsg.on_right', index=4,
      number=5, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=773,
  serialized_end=895,
)


_BUSMESSAGE = _descriptor.Descriptor(
  name='BusMessage',
  full_name='src.BusMessage',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='heartbeat', full_name='src.BusMessage.heartbeat', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='stopMoving', full_name='src.BusMessage.stopMoving', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='movementEnded', full_name='src.BusMessage.movementEnded', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='encoderPosition', full_name='src.BusMessage.encoderPosition', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pidConfig', full_name='src.BusMessage.pidConfig', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='motionLimit', full_name='src.BusMessage.motionLimit', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='setMotionControlMode', full_name='src.BusMessage.setMotionControlMode', index=6,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='moveWheelAtSpeed', full_name='src.BusMessage.moveWheelAtSpeed', index=7,
      number=8, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='translate', full_name='src.BusMessage.translate', index=8,
      number=9, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='rotate', full_name='src.BusMessage.rotate', index=9,
      number=10, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='servo', full_name='src.BusMessage.servo', index=10,
      number=11, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pumpAndValve', full_name='src.BusMessage.pumpAndValve', index=11,
      number=12, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='laserSensor', full_name='src.BusMessage.laserSensor', index=12,
      number=13, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pressureSensor', full_name='src.BusMessage.pressureSensor', index=13,
      number=14, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='message_content', full_name='src.BusMessage.message_content',
      index=0, containing_type=None, fields=[]),
  ],
  serialized_start=898,
  serialized_end=1557,
)

_BUSMESSAGE.fields_by_name['heartbeat'].message_type = _HEARTBEATMSG
_BUSMESSAGE.fields_by_name['stopMoving'].message_type = _STOPMOVINGMSG
_BUSMESSAGE.fields_by_name['movementEnded'].message_type = _MOVEMENTENDEDMSG
_BUSMESSAGE.fields_by_name['encoderPosition'].message_type = _ENCODERPOSITIONMSG
_BUSMESSAGE.fields_by_name['pidConfig'].message_type = _PIDCONFIGMSG
_BUSMESSAGE.fields_by_name['motionLimit'].message_type = _MOTIONLIMITMSG
_BUSMESSAGE.fields_by_name['setMotionControlMode'].message_type = _SETMOTIONCONTROLMODEMSG
_BUSMESSAGE.fields_by_name['moveWheelAtSpeed'].message_type = _MOVEWHEELATSPEEDMSG
_BUSMESSAGE.fields_by_name['translate'].message_type = _TRANSLATEMSG
_BUSMESSAGE.fields_by_name['rotate'].message_type = _ROTATEMSG
_BUSMESSAGE.fields_by_name['servo'].message_type = _SERVOMSG
_BUSMESSAGE.fields_by_name['pumpAndValve'].message_type = _PUMPANDVALVEMSG
_BUSMESSAGE.fields_by_name['laserSensor'].message_type = _LASERSENSORMSG
_BUSMESSAGE.fields_by_name['pressureSensor'].message_type = _PRESSURESENSORMSG
_BUSMESSAGE.oneofs_by_name['message_content'].fields.append(
  _BUSMESSAGE.fields_by_name['heartbeat'])
_BUSMESSAGE.fields_by_name['heartbeat'].containing_oneof = _BUSMESSAGE.oneofs_by_name['message_content']
_BUSMESSAGE.oneofs_by_name['message_content'].fields.append(
  _BUSMESSAGE.fields_by_name['stopMoving'])
_BUSMESSAGE.fields_by_name['stopMoving'].containing_oneof = _BUSMESSAGE.oneofs_by_name['message_content']
_BUSMESSAGE.oneofs_by_name['message_content'].fields.append(
  _BUSMESSAGE.fields_by_name['movementEnded'])
_BUSMESSAGE.fields_by_name['movementEnded'].containing_oneof = _BUSMESSAGE.oneofs_by_name['message_content']
_BUSMESSAGE.oneofs_by_name['message_content'].fields.append(
  _BUSMESSAGE.fields_by_name['encoderPosition'])
_BUSMESSAGE.fields_by_name['encoderPosition'].containing_oneof = _BUSMESSAGE.oneofs_by_name['message_content']
_BUSMESSAGE.oneofs_by_name['message_content'].fields.append(
  _BUSMESSAGE.fields_by_name['pidConfig'])
_BUSMESSAGE.fields_by_name['pidConfig'].containing_oneof = _BUSMESSAGE.oneofs_by_name['message_content']
_BUSMESSAGE.oneofs_by_name['message_content'].fields.append(
  _BUSMESSAGE.fields_by_name['motionLimit'])
_BUSMESSAGE.fields_by_name['motionLimit'].containing_oneof = _BUSMESSAGE.oneofs_by_name['message_content']
_BUSMESSAGE.oneofs_by_name['message_content'].fields.append(
  _BUSMESSAGE.fields_by_name['setMotionControlMode'])
_BUSMESSAGE.fields_by_name['setMotionControlMode'].containing_oneof = _BUSMESSAGE.oneofs_by_name['message_content']
_BUSMESSAGE.oneofs_by_name['message_content'].fields.append(
  _BUSMESSAGE.fields_by_name['moveWheelAtSpeed'])
_BUSMESSAGE.fields_by_name['moveWheelAtSpeed'].containing_oneof = _BUSMESSAGE.oneofs_by_name['message_content']
_BUSMESSAGE.oneofs_by_name['message_content'].fields.append(
  _BUSMESSAGE.fields_by_name['translate'])
_BUSMESSAGE.fields_by_name['translate'].containing_oneof = _BUSMESSAGE.oneofs_by_name['message_content']
_BUSMESSAGE.oneofs_by_name['message_content'].fields.append(
  _BUSMESSAGE.fields_by_name['rotate'])
_BUSMESSAGE.fields_by_name['rotate'].containing_oneof = _BUSMESSAGE.oneofs_by_name['message_content']
_BUSMESSAGE.oneofs_by_name['message_content'].fields.append(
  _BUSMESSAGE.fields_by_name['servo'])
_BUSMESSAGE.fields_by_name['servo'].containing_oneof = _BUSMESSAGE.oneofs_by_name['message_content']
_BUSMESSAGE.oneofs_by_name['message_content'].fields.append(
  _BUSMESSAGE.fields_by_name['pumpAndValve'])
_BUSMESSAGE.fields_by_name['pumpAndValve'].containing_oneof = _BUSMESSAGE.oneofs_by_name['message_content']
_BUSMESSAGE.oneofs_by_name['message_content'].fields.append(
  _BUSMESSAGE.fields_by_name['laserSensor'])
_BUSMESSAGE.fields_by_name['laserSensor'].containing_oneof = _BUSMESSAGE.oneofs_by_name['message_content']
_BUSMESSAGE.oneofs_by_name['message_content'].fields.append(
  _BUSMESSAGE.fields_by_name['pressureSensor'])
_BUSMESSAGE.fields_by_name['pressureSensor'].containing_oneof = _BUSMESSAGE.oneofs_by_name['message_content']
DESCRIPTOR.message_types_by_name['HeartbeatMsg'] = _HEARTBEATMSG
DESCRIPTOR.message_types_by_name['StopMovingMsg'] = _STOPMOVINGMSG
DESCRIPTOR.message_types_by_name['MovementEndedMsg'] = _MOVEMENTENDEDMSG
DESCRIPTOR.message_types_by_name['EncoderPositionMsg'] = _ENCODERPOSITIONMSG
DESCRIPTOR.message_types_by_name['MotionLimitMsg'] = _MOTIONLIMITMSG
DESCRIPTOR.message_types_by_name['PIDConfigMsg'] = _PIDCONFIGMSG
DESCRIPTOR.message_types_by_name['SetMotionControlModeMsg'] = _SETMOTIONCONTROLMODEMSG
DESCRIPTOR.message_types_by_name['MoveWheelAtSpeedMsg'] = _MOVEWHEELATSPEEDMSG
DESCRIPTOR.message_types_by_name['TranslateMsg'] = _TRANSLATEMSG
DESCRIPTOR.message_types_by_name['RotateMsg'] = _ROTATEMSG
DESCRIPTOR.message_types_by_name['ServoMsg'] = _SERVOMSG
DESCRIPTOR.message_types_by_name['PumpAndValveMsg'] = _PUMPANDVALVEMSG
DESCRIPTOR.message_types_by_name['LaserSensorMsg'] = _LASERSENSORMSG
DESCRIPTOR.message_types_by_name['PressureSensorMsg'] = _PRESSURESENSORMSG
DESCRIPTOR.message_types_by_name['BusMessage'] = _BUSMESSAGE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

HeartbeatMsg = _reflection.GeneratedProtocolMessageType('HeartbeatMsg', (_message.Message,), {
  'DESCRIPTOR' : _HEARTBEATMSG,
  '__module__' : 'outech_pb2'
  # @@protoc_insertion_point(class_scope:src.HeartbeatMsg)
  })
_sym_db.RegisterMessage(HeartbeatMsg)

StopMovingMsg = _reflection.GeneratedProtocolMessageType('StopMovingMsg', (_message.Message,), {
  'DESCRIPTOR' : _STOPMOVINGMSG,
  '__module__' : 'outech_pb2'
  # @@protoc_insertion_point(class_scope:src.StopMovingMsg)
  })
_sym_db.RegisterMessage(StopMovingMsg)

MovementEndedMsg = _reflection.GeneratedProtocolMessageType('MovementEndedMsg', (_message.Message,), {
  'DESCRIPTOR' : _MOVEMENTENDEDMSG,
  '__module__' : 'outech_pb2'
  # @@protoc_insertion_point(class_scope:src.MovementEndedMsg)
  })
_sym_db.RegisterMessage(MovementEndedMsg)

EncoderPositionMsg = _reflection.GeneratedProtocolMessageType('EncoderPositionMsg', (_message.Message,), {
  'DESCRIPTOR' : _ENCODERPOSITIONMSG,
  '__module__' : 'outech_pb2'
  # @@protoc_insertion_point(class_scope:src.EncoderPositionMsg)
  })
_sym_db.RegisterMessage(EncoderPositionMsg)

MotionLimitMsg = _reflection.GeneratedProtocolMessageType('MotionLimitMsg', (_message.Message,), {
  'DESCRIPTOR' : _MOTIONLIMITMSG,
  '__module__' : 'outech_pb2'
  # @@protoc_insertion_point(class_scope:src.MotionLimitMsg)
  })
_sym_db.RegisterMessage(MotionLimitMsg)

PIDConfigMsg = _reflection.GeneratedProtocolMessageType('PIDConfigMsg', (_message.Message,), {
  'DESCRIPTOR' : _PIDCONFIGMSG,
  '__module__' : 'outech_pb2'
  # @@protoc_insertion_point(class_scope:src.PIDConfigMsg)
  })
_sym_db.RegisterMessage(PIDConfigMsg)

SetMotionControlModeMsg = _reflection.GeneratedProtocolMessageType('SetMotionControlModeMsg', (_message.Message,), {
  'DESCRIPTOR' : _SETMOTIONCONTROLMODEMSG,
  '__module__' : 'outech_pb2'
  # @@protoc_insertion_point(class_scope:src.SetMotionControlModeMsg)
  })
_sym_db.RegisterMessage(SetMotionControlModeMsg)

MoveWheelAtSpeedMsg = _reflection.GeneratedProtocolMessageType('MoveWheelAtSpeedMsg', (_message.Message,), {
  'DESCRIPTOR' : _MOVEWHEELATSPEEDMSG,
  '__module__' : 'outech_pb2'
  # @@protoc_insertion_point(class_scope:src.MoveWheelAtSpeedMsg)
  })
_sym_db.RegisterMessage(MoveWheelAtSpeedMsg)

TranslateMsg = _reflection.GeneratedProtocolMessageType('TranslateMsg', (_message.Message,), {
  'DESCRIPTOR' : _TRANSLATEMSG,
  '__module__' : 'outech_pb2'
  # @@protoc_insertion_point(class_scope:src.TranslateMsg)
  })
_sym_db.RegisterMessage(TranslateMsg)

RotateMsg = _reflection.GeneratedProtocolMessageType('RotateMsg', (_message.Message,), {
  'DESCRIPTOR' : _ROTATEMSG,
  '__module__' : 'outech_pb2'
  # @@protoc_insertion_point(class_scope:src.RotateMsg)
  })
_sym_db.RegisterMessage(RotateMsg)

ServoMsg = _reflection.GeneratedProtocolMessageType('ServoMsg', (_message.Message,), {
  'DESCRIPTOR' : _SERVOMSG,
  '__module__' : 'outech_pb2'
  # @@protoc_insertion_point(class_scope:src.ServoMsg)
  })
_sym_db.RegisterMessage(ServoMsg)

PumpAndValveMsg = _reflection.GeneratedProtocolMessageType('PumpAndValveMsg', (_message.Message,), {
  'DESCRIPTOR' : _PUMPANDVALVEMSG,
  '__module__' : 'outech_pb2'
  # @@protoc_insertion_point(class_scope:src.PumpAndValveMsg)
  })
_sym_db.RegisterMessage(PumpAndValveMsg)

LaserSensorMsg = _reflection.GeneratedProtocolMessageType('LaserSensorMsg', (_message.Message,), {
  'DESCRIPTOR' : _LASERSENSORMSG,
  '__module__' : 'outech_pb2'
  # @@protoc_insertion_point(class_scope:src.LaserSensorMsg)
  })
_sym_db.RegisterMessage(LaserSensorMsg)

PressureSensorMsg = _reflection.GeneratedProtocolMessageType('PressureSensorMsg', (_message.Message,), {
  'DESCRIPTOR' : _PRESSURESENSORMSG,
  '__module__' : 'outech_pb2'
  # @@protoc_insertion_point(class_scope:src.PressureSensorMsg)
  })
_sym_db.RegisterMessage(PressureSensorMsg)

BusMessage = _reflection.GeneratedProtocolMessageType('BusMessage', (_message.Message,), {
  'DESCRIPTOR' : _BUSMESSAGE,
  '__module__' : 'outech_pb2'
  # @@protoc_insertion_point(class_scope:src.BusMessage)
  })
_sym_db.RegisterMessage(BusMessage)


# @@protoc_insertion_point(module_scope)