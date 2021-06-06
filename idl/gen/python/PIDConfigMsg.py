# automatically generated by the FlatBuffers compiler, do not modify

# namespace: python

import flatbuffers
from flatbuffers.compat import import_numpy
np = import_numpy()

class PIDConfigMsg(object):
    __slots__ = ['_tab']

    @classmethod
    def GetRootAsPIDConfigMsg(cls, buf, offset):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, offset)
        x = PIDConfigMsg()
        x.Init(buf, n + offset)
        return x

    # PIDConfigMsg
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # PIDConfigMsg
    def PidSpeedLeft(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            x = o + self._tab.Pos
            from idl.gen.python.PIDCoefficients import PIDCoefficients
            obj = PIDCoefficients()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # PIDConfigMsg
    def PidSpeedRight(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            x = o + self._tab.Pos
            from idl.gen.python.PIDCoefficients import PIDCoefficients
            obj = PIDCoefficients()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # PIDConfigMsg
    def PidPositionLeft(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(8))
        if o != 0:
            x = o + self._tab.Pos
            from idl.gen.python.PIDCoefficients import PIDCoefficients
            obj = PIDCoefficients()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

    # PIDConfigMsg
    def PidPositionRight(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(10))
        if o != 0:
            x = o + self._tab.Pos
            from idl.gen.python.PIDCoefficients import PIDCoefficients
            obj = PIDCoefficients()
            obj.Init(self._tab.Bytes, x)
            return obj
        return None

def PIDConfigMsgStart(builder): builder.StartObject(4)
def PIDConfigMsgAddPidSpeedLeft(builder, pidSpeedLeft): builder.PrependStructSlot(0, flatbuffers.number_types.UOffsetTFlags.py_type(pidSpeedLeft), 0)
def PIDConfigMsgAddPidSpeedRight(builder, pidSpeedRight): builder.PrependStructSlot(1, flatbuffers.number_types.UOffsetTFlags.py_type(pidSpeedRight), 0)
def PIDConfigMsgAddPidPositionLeft(builder, pidPositionLeft): builder.PrependStructSlot(2, flatbuffers.number_types.UOffsetTFlags.py_type(pidPositionLeft), 0)
def PIDConfigMsgAddPidPositionRight(builder, pidPositionRight): builder.PrependStructSlot(3, flatbuffers.number_types.UOffsetTFlags.py_type(pidPositionRight), 0)
def PIDConfigMsgEnd(builder): return builder.EndObject()
