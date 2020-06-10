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

# Big servo for high/low positions
SERVO_ANGLES_BIG_LOW = [40, 10, 30, 30, 30]  # left r right
SERVO_ANGLES_BIG_HIGH = [180, 150, 160, 160, 160]  # left r right

# Middle knee servo for high/lower position
SERVO_ANGLES_MID_LOW = [0, 0, 0, 0, 0]  # left r right
SERVO_ANGLES_MID_HIGH = [120, 120, 120, 120, 120]  # left r right

# Gripper open/close
SERVO_ANGLES_OPEN = [180, 180, 180, 180, 180]
SERVO_ANGLES_CLOSE = [20, 20, 20, 20, 20]


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
    data = msg.SerializeToString()
    await sock.send(data)


async def send_orders_servos(sockets, servo_id, angles):
    for i in range(len(sockets)):
        await send_order(sockets[i], ORDER_SERVO, servo_id, angles[i])


async def main():
    boards = [ISOTPSocketAdapter(ISOTPAddress("can0", get_rx_id(SERVOS_ID_BASE+i), get_tx_id(SERVOS_ID_BASE+i)), "servo"+str(i)) for i in range(5)]
    for b in boards:
        print(b, "ready")
    print("Adapters ready")

    help_str = "Orders: \n\tO= open grabbers, T = take cups, R = reset position, ? = this message"
    print(help_str)
    try:
        async def main_loop():
            while True:
                await asyncio.sleep(0.1)
                order = input()
                if order == "?":
                    print(help_str)
                else:
                    if order == "O":
                        await send_orders_servos(boards, 0, SERVO_ANGLES_OPEN)
                    elif order == "T":
                        await send_orders_servos(boards, 2, SERVO_ANGLES_BIG_LOW)

                        await asyncio.sleep(1)

                        await send_orders_servos(boards, 1, SERVO_ANGLES_MID_LOW)

                        await asyncio.sleep(1)
                        await send_orders_servos(boards, 0, SERVO_ANGLES_CLOSE)

                    elif order == "R":
                        await send_orders_servos(boards, 1, SERVO_ANGLES_MID_HIGH)
                        await send_orders_servos(boards, 2, SERVO_ANGLES_BIG_HIGH)
        task = asyncio.get_event_loop().create_task(main_loop())    
    except KeyboardInterrupt:
        print("Stopping, close socket")
        for b in boards:
           del b 
    tasks = []
    for b in boards:
        tasks.append(asyncio.get_event_loop().create_task(b.run()))
    await asyncio.sleep(10000)
if __name__ == "__main__":
    asyncio.run(main())
