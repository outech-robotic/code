import asyncio
import socket
from os import system
from proto.gen.python.outech_pb2 import BusMessage, ServoMsg, PumpAndValveMsg
import binascii
from time import sleep
from highlevel.adapter.socket.isotp import ISOTPAddress, ISOTPSocketAdapter

SERVOS_PORT_BASE = 14000+0x10  # base port + servo id offset
SERVOS_ID_BASE = 0x010  # base CAN ID

MSG_SERVO = BusMessage(servo=ServoMsg())
MSG_PUMP = BusMessage(pumpAndValve=PumpAndValveMsg())

ORDER_SERVO = 0
ORDER_PUMP = 1

servo_0_open = 180
servo_0_close = 50

servo_1_up = 75
servo_1_down = 180

servo_2_up = 120
servo_2_take = 55
servo_2_put = 60

# OFFSETS PER SERVO
# Right of robot to left
# Big servo for high/low positions
OFFSET_2_UP  = [18, 15, 18, 0, 0]
OFFSET_2_TAKE= [0, -5, -5, -20, -16]
OFFSET_2_PUT = [0, 0, 0, 0, 0]

# Middle knee servo for high/lower position
OFFSET_1_UP   = [0, 0, 0, 0, 0]
OFFSET_1_DOWN = [0, 0, 0, 0, 0]

# Gripper open/close
OFFSET_0_OPEN  = [0, 0, 0, 0, 0]
OFFSET_0_CLOSE = [0, 0, 0, 0, 0]


def get_rx_id(id):
    return id*2+1

def get_tx_id(id):
    return id*2

async def send_order(sock, msg_type, msg_id, value):
    if msg_type == ORDER_SERVO:
        print("Servo order : ", msg_id, value)
        msg = MSG_SERVO
        msg.servo.id = msg_id
        msg.servo.angle = value

    elif msg_type == ORDER_PUMP:
        print("Pump order: ", msg_id, value)
        msg = MSG_PUMP
        msg.pumpAndValve.id = msg_id
        print(value)
        if value == 1:
            msg.pumpAndValve.on = True
        elif value == 0:
            msg.pumpAndValve.on = False
        else:
            print("Error : For a pump board, the value to send is 1 or 0 only")
            return
    else:
        print("Error : type of order should be ORDER_SERVO or ORDER_PUMP")
        return
    print(sock, msg_type, msg_id, value, msg)
    data = msg.SerializeToString()
    await sock.send(data)


async def send_orders_servos(sockets, servo_id, angle, offsets):
    for i in range(len(sockets)):
        await send_order(sockets[i], ORDER_SERVO, servo_id, angle+offsets[i])

async def loop_angles(boards, start, end):
    msg = MSG_SERVO 
    for i in range(start, end):
        msg.servo.angle = i
        for b in boards:
            for id in range(3):
                msg.servo.id = id
                payload = msg.SerializeToString()
                await b.send(payload)
        await asyncio.sleep(0.1)

async def main():
    boards = [ISOTPSocketAdapter(ISOTPAddress("can0", get_rx_id(SERVOS_ID_BASE+i), get_tx_id(SERVOS_ID_BASE+i)), "servo"+str(i)) for i in range(5)]
    for b in boards:
        print(b, "ready")
    print("Adapters ready")

    help_str = "Orders: \n\to= open grabbers, t = take cups, p = put cups, r = reset position (up), ? = this message"
    print(help_str)

    async def main_test_angles():
        while True:
            await asyncio.sleep(0.1)
            print("Heyyah")
            servo = int(input("Entrer un id de servo: 0/1/2\n"))
            val = int(input("Entrer un angle: 0...180\n"))
            print(servo, val)
            if 0 <= val <= 180:
                print(val)
                vals = [val for _ in boards]
                print(vals)
                await send_orders_servos(boards, servo, vals)
            else:
                print("Wrong value")
    async def main_loop():
        # while True:
        #     await loop_angles(boards, 90, 91)
        while True:
            await asyncio.sleep(0.1)
            order = input("Entrer ordre: r/o/t/p/?\n")
            if order == "?":
                print(help_str)
            else:
                if order == "o":
                    await send_orders_servos(boards, 0, servo_0_open, OFFSET_0_OPEN)
                elif order == "t":
                    await send_orders_servos(boards, 2, servo_2_take, OFFSET_2_TAKE)

                    await asyncio.sleep(1)

                    await send_orders_servos(boards, 1, servo_1_down, OFFSET_1_DOWN)

                    await asyncio.sleep(1)
                    await send_orders_servos(boards, 0, servo_0_close, OFFSET_0_CLOSE)
                elif order == "p":
                    await send_orders_servos(boards, 2, servo_2_put, OFFSET_2_PUT)

                    await asyncio.sleep(1)

                    await send_orders_servos(boards, 1, servo_1_down, OFFSET_1_DOWN)

                    await asyncio.sleep(1)

                    await send_orders_servos(boards, 0, servo_0_open, OFFSET_0_OPEN)

                    await asyncio.sleep(1)

                    await send_orders_servos(boards, 2, servo_2_up, OFFSET_2_UP)
                elif order == "r":
                    await send_orders_servos(boards, 1, servo_1_up, OFFSET_1_UP)
                    await send_orders_servos(boards, 2, servo_2_up, OFFSET_2_UP)
    main_task = asyncio.get_event_loop().create_task(main_loop())    
    tasks = []
    for b in boards:
        tasks.append(asyncio.get_event_loop().create_task(b.run()))
    await asyncio.sleep(10000)
if __name__ == "__main__":
    asyncio.run(main())
