# automatically generated by the FlatBuffers compiler, do not modify

# namespace: python

import flatbuffers
from flatbuffers.compat import import_numpy
np = import_numpy()

class DebugLog(object):
    __slots__ = ['_tab']

    @classmethod
    def GetRootAsDebugLog(cls, buf, offset):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, offset)
        x = DebugLog()
        x.Init(buf, n + offset)
        return x

    # DebugLog
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # DebugLog
    def Content(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.String(o + self._tab.Pos)
        return None

def DebugLogStart(builder): builder.StartObject(1)
def DebugLogAddContent(builder, content): builder.PrependUOffsetTRelativeSlot(0, flatbuffers.number_types.UOffsetTFlags.py_type(content), 0)
def DebugLogEnd(builder): return builder.EndObject()
