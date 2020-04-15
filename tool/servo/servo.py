import socket
from os import system
from proto.gen.python.outech_pb2 import BusMessage, ServoMsg, PumpAndValveMsg
import binascii
from time import sleep

SERVOS_PORT_BASE = 32000  # base port
SERVOS_ID_BASE = 0x010  # base CAN ID
TX_ID_OFFSET = 0x400  # Offset to board ID for TX ID (most significant bit at 1)

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


def send_order(sock, msg_type, msg_id, value):
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
    data_sent = b'<' + binascii.hexlify(data) + b'>\n'
    sock.send(data_sent)


def send_orders_servos(sockets, servo_id, angles):
    for i in range(len(sockets)):
        send_order(sockets[i], ORDER_SERVO, servo_id, angles[i])


def main():
    board = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    board.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    boards = [socket.socket(socket.AF_INET, socket.SOCK_STREAM) for _ in range(5)]

    for i, sock in enumerate(boards):
        port = SERVOS_PORT_BASE + i
        can_rx = SERVOS_ID_BASE + i
        can_tx = can_rx | TX_ID_OFFSET
        can_rx = format(can_rx, 'x')
        can_tx = format(can_tx, 'x')
        print("CAN IDs:", can_rx, can_tx)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        print("Starting isotpserver : port/rx/tx:", port, can_rx, can_tx)
        system("isotpserver -l " + str(port) + " -s " + can_tx + " -d " + can_rx + " can0 &")
        sleep(0.1)
        print("Trying to connect to localhost:", port)
        sock.connect(("localhost", port))

    print("Sockets ready")

    help_str = "Orders: \n\tO= open grabbers, T = take cups, R = reset position, ? = this message"
    print(help_str)

    try:
        while True:
            order = input()
            if order == "?":
                print(help_str)
            else:
                if order == "O":
                    send_orders_servos(boards, 0, SERVO_ANGLES_OPEN)
                elif order == "T":
                    send_orders_servos(boards, 2, SERVO_ANGLES_BIG_LOW)

                    sleep(1)

                    send_orders_servos(boards, 1, SERVO_ANGLES_MID_LOW)

                    sleep(1)
                    send_orders_servos(boards, 0, SERVO_ANGLES_CLOSE)

                elif order == "R":
                    send_orders_servos(boards, 1, SERVO_ANGLES_MID_HIGH)
                    send_orders_servos(boards, 2, SERVO_ANGLES_BIG_HIGH)

    except KeyboardInterrupt:
        print("Stopping, close socket")
        board.close()


if __name__ == "__main__":
    main()
