# On windows :
# Install  protoc-3.11.4-win64.zip from https://github.com/protocolbuffers/protobuf/releases
# protoc --python_out=proto proto.pb
# pip install protobuf
#
# from src.proto.proto.pb_pb2 import *
#
# epm = BusMessage(encoderPosition=EncoderPositionMsg())
# print(epm.IsInitialized())
# print(epm.__str__())
#
# epm.encoderPosition.left_tick = 10
# epm.encoderPosition.right_tick = 10
# print(epm.IsInitialized())
# print(epm.__str__())
#
# epm.Clear()
# print(epm.IsInitialized())
# print(epm.__str__())
#
# epm.encoderPosition.left_tick = 10
# epm.encoderPosition.right_tick = 10
# print(epm.IsInitialized())
# print(epm.__str__())
#
# sepm = epm.SerializeToString()
# print("Serialized : ", sepm)
#
# repm = BusMessage()
# repm.ParseFromString(sepm)
# print(repm.WhichOneof("message_content"))
#
# print("Deserialized : ", repm.__str__())
