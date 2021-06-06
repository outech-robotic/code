# automatically generated by the FlatBuffers compiler, do not modify

# namespace: python

import flatbuffers
from flatbuffers.compat import import_numpy
np = import_numpy()

class BusMessage(object):
    __slots__ = ['_tab']

    @classmethod
    def GetRootAsBusMessage(cls, buf, offset):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, offset)
        x = BusMessage()
        x.Init(buf, n + offset)
        return x

    # BusMessage
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # BusMessage
    def ContentType(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Uint8Flags, o + self._tab.Pos)
        return 0

    # BusMessage
    def Content(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            from flatbuffers.table import Table
            obj = Table(bytearray(), 0)
            self._tab.Union(obj, o)
            return obj
        return None

def BusMessageStart(builder): builder.StartObject(2)
def BusMessageAddContentType(builder, contentType): builder.PrependUint8Slot(0, contentType, 0)
def BusMessageAddContent(builder, content): builder.PrependUOffsetTRelativeSlot(1, flatbuffers.number_types.UOffsetTFlags.py_type(content), 0)
def BusMessageEnd(builder): return builder.EndObject()
