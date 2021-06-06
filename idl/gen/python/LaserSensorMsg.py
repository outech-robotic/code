# automatically generated by the FlatBuffers compiler, do not modify

# namespace: python

import flatbuffers
from flatbuffers.compat import import_numpy
np = import_numpy()

class LaserSensorMsg(object):
    __slots__ = ['_tab']

    @classmethod
    def GetRootAsLaserSensorMsg(cls, buf, offset):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, offset)
        x = LaserSensorMsg()
        x.Init(buf, n + offset)
        return x

    # LaserSensorMsg
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # LaserSensorMsg
    def DistanceFrontLeft(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Uint32Flags, o + self._tab.Pos)
        return 0

    # LaserSensorMsg
    def DistanceFrontRight(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Uint32Flags, o + self._tab.Pos)
        return 0

    # LaserSensorMsg
    def DistanceBackLeft(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(8))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Uint32Flags, o + self._tab.Pos)
        return 0

    # LaserSensorMsg
    def DistanceBackRight(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(10))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Uint32Flags, o + self._tab.Pos)
        return 0

def LaserSensorMsgStart(builder): builder.StartObject(4)
def LaserSensorMsgAddDistanceFrontLeft(builder, distanceFrontLeft): builder.PrependUint32Slot(0, distanceFrontLeft, 0)
def LaserSensorMsgAddDistanceFrontRight(builder, distanceFrontRight): builder.PrependUint32Slot(1, distanceFrontRight, 0)
def LaserSensorMsgAddDistanceBackLeft(builder, distanceBackLeft): builder.PrependUint32Slot(2, distanceBackLeft, 0)
def LaserSensorMsgAddDistanceBackRight(builder, distanceBackRight): builder.PrependUint32Slot(3, distanceBackRight, 0)
def LaserSensorMsgEnd(builder): return builder.EndObject()
