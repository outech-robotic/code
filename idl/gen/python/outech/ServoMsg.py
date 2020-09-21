# automatically generated by the FlatBuffers compiler, do not modify

# namespace: outech

import flatbuffers
from flatbuffers.compat import import_numpy
np = import_numpy()

class ServoMsg(object):
    __slots__ = ['_tab']

    @classmethod
    def GetRootAsServoMsg(cls, buf, offset):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, offset)
        x = ServoMsg()
        x.Init(buf, n + offset)
        return x

    # ServoMsg
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # ServoMsg
    def Id(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Uint32Flags, o + self._tab.Pos)
        return 0

    # ServoMsg
    def Angle(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Int32Flags, o + self._tab.Pos)
        return 0

def ServoMsgStart(builder): builder.StartObject(2)
def ServoMsgAddId(builder, id): builder.PrependUint32Slot(0, id, 0)
def ServoMsgAddAngle(builder, angle): builder.PrependInt32Slot(1, angle, 0)
def ServoMsgEnd(builder): return builder.EndObject()
