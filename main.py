import random
import string
from math import sin, pi
from threading import Thread
from time import sleep, time
from typing import Optional

from flask import Flask
from flask_socketio import SocketIO
from views import register_views, InterfaceAdapter, PID

import can
import struct


CAN_BOARD_ID_WIDTH = 5
CAN_MSG_WIDTH  = 9
CAN_BOARD_ID_MOTOR = 15
CAN_CHANNEL_MOTOR  = 0b00
CAN_MSG_SPEED   = 0b1110
CAN_MSG_POS     = 0b0010
CAN_KP_ID = 0b1100
CAN_KI_ID = 0b1101
CAN_KD_ID = 0b1110
PID_LEFT_SPEED = 0
PID_LEFT_POS = 1
PID_RIGHT_SPEED = 2
PID_RIGHT_POS = 3

fmt_motor_set_speed = struct.Struct('<ll')  # 32b + 32b signe
fmt_motor_set_pos   = struct.Struct('<ll')
fmt_motor_set_pid   = struct.Struct('<BL')  # 8b + 32b non signes

WHEEL_DIAMETER = 60  # en mm ; 2400 ticks par tour donc par 2*pi*60 mm
DISTANCE_BETWEEN_WHEELS = 300  # en mm


# CAN
def send_packet(channel, message_id, board, data=[]):
    can_pkt_id = ((channel << CAN_MSG_WIDTH) | (message_id << CAN_BOARD_ID_WIDTH) | board)
    with can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000) as bus:
        msg = can.Message(
            arbitration_id=can_pkt_id, data=data, is_extended_id=False
        )
        try:
            bus.send(msg)
            print(f"message_id sent on {bus.channel_info}")
        except can.CanError:
            print(can.CanError)
            print("message_id NOT sent")
            
# Flask
app = Flask(__name__)
app.config['TEMPLATES_AUTO_RELOAD'] = True
app.config['SECRET_KEY'] = ''.join(random.choice(string.printable) for i in range(64))
socketio = SocketIO(app, cors_allowed_origins="*")

class Adapter(InterfaceAdapter):
    """ La classe qu'il faut implem pour s'interfacer avec la page web. """

    def __init__(self, socketio: SocketIO):
        super(Adapter, self).__init__(socketio)  # Il faut garder cette ligne.

        # A la place de cette fonction et du thread, on met le code qui recoit les msg CAN et on
        # appelle les fonctions .push_*_*
        def f():
            while True:
                sleep(1 / 100)
                t = int(time() * 1000)
                v = 100 + sin(time() * 1) * 10
                # 1er argument le temps (time.time()), 2eme la valeur, 3eme la consigne.

                self.push_speed_left(t, 10 + v, 100)
                self.push_pos_left(t, -v, -100)

                self.push_speed_right(t, 10 + v, 100)
                self.push_pos_right(t, -v, -100)

        Thread(target=f).start()

    def on_pid_submission(self, speed_left: PID, speed_right: PID, pos_left: PID,
                          pos_right: PID) -> None:
        # Ici tu implem l'envoi des paquets sur le CAN.
        print(f"Got PID {pos_left} {pos_right} {speed_left} {speed_right}")

        pos_left.p = round(pos_left.p*65535)
        pos_left.i = round(pos_left.i*65535)
        pos_left.d = round(pos_left.d*65535)

        pos_right.p = round(pos_right.p*65535)
        pos_right.i = round(pos_right.i*65535)
        pos_right.d = round(pos_right.d*65535)

        speed_left.p = round(speed_left.p*65535)
        speed_left.i = round(speed_left.i*65535)
        speed_left.d = round(speed_left.d*65535)

        speed_right.p = round(speed_right.p*65535)
        speed_right.i = round(speed_right.i*65535)
        speed_right.d = round(speed_right.d*65535)

        send_packet(CAN_CHANNEL_MOTOR, CAN_KP_ID, CAN_BOARD_ID_MOTOR, fmt_motor_set_pid.pack(PID_LEFT_POS, int(pos_left.p)))
        send_packet(CAN_CHANNEL_MOTOR, CAN_KI_ID, CAN_BOARD_ID_MOTOR, fmt_motor_set_pid.pack(PID_LEFT_POS, int(pos_left.i)))
        send_packet(CAN_CHANNEL_MOTOR, CAN_KD_ID, CAN_BOARD_ID_MOTOR, fmt_motor_set_pid.pack(PID_LEFT_POS, int(pos_left.d)))

        send_packet(CAN_CHANNEL_MOTOR, CAN_KP_ID, CAN_BOARD_ID_MOTOR, fmt_motor_set_pid.pack(PID_RIGHT_POS, int(pos_right.p)))
        send_packet(CAN_CHANNEL_MOTOR, CAN_KI_ID, CAN_BOARD_ID_MOTOR, fmt_motor_set_pid.pack(PID_RIGHT_POS, int(pos_right.i)))
        send_packet(CAN_CHANNEL_MOTOR, CAN_KD_ID, CAN_BOARD_ID_MOTOR, fmt_motor_set_pid.pack(PID_RIGHT_POS, int(pos_right.d)))

        send_packet(CAN_CHANNEL_MOTOR, CAN_KP_ID, CAN_BOARD_ID_MOTOR, fmt_motor_set_pid.pack(PID_LEFT_SPEED, int(speed_left.p)))
        send_packet(CAN_CHANNEL_MOTOR, CAN_KI_ID, CAN_BOARD_ID_MOTOR, fmt_motor_set_pid.pack(PID_LEFT_SPEED, int(speed_left.i)))
        send_packet(CAN_CHANNEL_MOTOR, CAN_KD_ID, CAN_BOARD_ID_MOTOR, fmt_motor_set_pid.pack(PID_LEFT_SPEED, int(speed_left.d)))

        send_packet(CAN_CHANNEL_MOTOR, CAN_KP_ID, CAN_BOARD_ID_MOTOR, fmt_motor_set_pid.pack(PID_RIGHT_SPEED, int(speed_right.p)))
        send_packet(CAN_CHANNEL_MOTOR, CAN_KI_ID, CAN_BOARD_ID_MOTOR, fmt_motor_set_pid.pack(PID_RIGHT_SPEED, int(speed_right.i)))
        send_packet(CAN_CHANNEL_MOTOR, CAN_KD_ID, CAN_BOARD_ID_MOTOR, fmt_motor_set_pid.pack(PID_RIGHT_SPEED, int(speed_right.d)))

    def on_order_submission(self, speed: Optional[float], position: Optional[float],
                            angle: Optional[float]):
        # Ici tu implem l'envoi des paquets sur le CAN.
        print(f"Got order {speed} {position} {angle}")  # position en mm, speed en mm/s, angle en degré
        # Conversion en ticks
        coeff = 2400/(2*pi*60)

        # Cas 1 : juste translation
        if angle is None or angle == 0 or angle == 0.0:
            if speed is not None:
                speed_ticks = speed * coeff
                send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_SPEED, CAN_BOARD_ID_MOTOR, fmt_motor_set_speed.pack(speed_ticks, speed_ticks))
            if position is not None:
                position_ticks = position * coeff
                send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_POS, CAN_BOARD_ID_MOTOR, fmt_motor_set_pos.pack(position_ticks, position_ticks))

        # Cas 2 : juste rotation
        else:
            angle_ticks = angle * DISTANCE_BETWEEN_WHEELS / 2
            if speed is not None:
                speed_ticks = speed * coeff
                send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_SPEED, CAN_BOARD_ID_MOTOR,
                            fmt_motor_set_speed.pack(speed_ticks, speed_ticks))
            if angle > 0:  # sens trigo
                send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_POS, CAN_BOARD_ID_MOTOR, fmt_motor_set_pos.pack(-angle_ticks, angle_ticks))  # roue gauche, roue droite
            if angle < 0:
                send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_POS, CAN_BOARD_ID_MOTOR, fmt_motor_set_pos.pack(+angle_ticks, -angle_ticks))  # roue gauche, roue droite


register_views(app, socketio, Adapter(socketio))

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)
