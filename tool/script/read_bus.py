import argparse
import asyncio
import json
import sys
import time

from google.protobuf import json_format
from google.protobuf.message import DecodeError

from highlevel.adapter.socket.isotp import ISOTPSocketAdapter, ISOTPAddress
from proto.gen.python.outech_pb2 import BusMessage

t_last = time.time()


async def main():
    global t_last
    parser = argparse.ArgumentParser(description='Read packets on the CAN bus.')

    parser.add_argument('device', metavar='device', type=str,
                        help='CAN device to use. Can be virtual or physical.')
    parser.add_argument('id_rx', metavar='id_rx', type=int,
                        help='CAN address to accept when receiving. Should be in [0, 1023]')
    parser.add_argument('id_tx', metavar='id_tx', type=int,
                        help='CAN address to transmit to. Should be in [0, 1023]')

    args = parser.parse_args()

    t_last = time.time()

    assert args.id_rx != args.id_tx
    assert len(args.device) > 0

    isotp = ISOTPSocketAdapter(
        address=ISOTPAddress(args.device, args.id_rx, args.id_tx),
        adapter_name="receiver"
    )

    async def callback(bytes, name):
        global t_last
        t = time.time()
        bus_message = BusMessage()
        try:
            bus_message.ParseFromString(bytes)
            printable_data = json_format.MessageToDict(bus_message,
                                                       including_default_value_fields=True)
            json_data = json.dumps(printable_data)
            sys.stdout.write(f'{(t - t_last) * 1000:10.3f} ms:"{name}" ' + json_data + '\n')
            sys.stdout.flush()
        except DecodeError:
            print("Protobuf couldn't decode this:", bytes)
        t_last = t

    isotp.register_callback(callback)

    try:
        await isotp.run(),
    except KeyboardInterrupt:
        print("KeyboardInterrupt")


if __name__ == '__main__':
    asyncio.run(main())
