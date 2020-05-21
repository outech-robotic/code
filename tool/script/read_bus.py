import argparse
import asyncio
import json
import sys

from google.protobuf import json_format

from highlevel.adapter.socket.socket_adapter import TCPSocketAdapter
from highlevel.util import tcp
from proto.gen.python.outech_pb2 import BusMessage


async def main():
    parser = argparse.ArgumentParser(description='Read packets on the CAN bus.')
    parser.add_argument('tcp_port', metavar='tcp_port', type=int,
                        help='isotpserver TCP port number')

    args = parser.parse_args()

    reader, writer = await tcp.get_reader_writer('localhost', args.tcp_port)
    adapter = TCPSocketAdapter(reader, writer, 'ug_le_bg')

    async def callback(bytes, _):
        bus_message = BusMessage()
        bus_message.ParseFromString(bytes)
        printable_data = json_format.MessageToDict(bus_message, including_default_value_fields=True)
        json_data = json.dumps(printable_data)
        sys.stdout.write(json_data + '\n')
        sys.stdout.flush()

    adapter.register_callback(callback)
    await adapter.run()


if __name__ == '__main__':
    asyncio.run(main())
