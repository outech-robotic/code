#!/usr/bin/python3
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
CAN_MSG_SPEED   = 0b1000
CAN_MSG_POS     = 0b0010
CAN_MSG_STOP    = 0b0000
CAN_KP_ID = 0b1100
CAN_KI_ID = 0b1101
CAN_KD_ID = 0b1110
PID_LEFT_SPEED = 0
PID_LEFT_POS = 1
PID_RIGHT_SPEED = 2
PID_RIGHT_POS = 3
COD_UPDATE_FREQ = 100 #Hz
fmt_motor_set_speed = struct.Struct('<ll')  # 32b + 32b signe
fmt_motor_set_pos   = struct.Struct('<ll')
fmt_motor_cod_pos   = struct.Struct('<ll')
fmt_motor_set_pid   = struct.Struct('<BL')  # 8b + 32b non signes

WHEEL_DIAMETER = 70  # en mm ; 2400 ticks par tour donc par 2*pi*60 mm
DISTANCE_BETWEEN_WHEELS = 365  # en mm


# CAN
def send_packet(channel, message_id, board, data=[]):
    can_pkt_id = ((channel << CAN_MSG_WIDTH) | (message_id << CAN_BOARD_ID_WIDTH) | board)
    with can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000) as bus:
        msg = can.Message(
            arbitration_id=can_pkt_id, data=data, is_extended_id=False
        )
        try:
            bus.send(msg)
            print(can_pkt_id, "sent on", bus.channel_info)
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
            start_t=time()
            last_left, last_right = 0, 0
            with can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000) as bus:
                for message in bus:
                    if (message.arbitration_id >>5) == 0b000011:
                        #sleep(1.0/COD_UPDATE_FREQ)
                        posl, posr = fmt_motor_cod_pos.unpack(message.data)
                        #print(posl, posr)
                        speedl, speedr = (posl-last_left)*COD_UPDATE_FREQ, (posr - last_right)*COD_UPDATE_FREQ
                        #print(speedl, speedr)
                        last_left, last_right = posl, posr
                        t = int((time()-start_t) * 1000)
                        self.push_speed_left(t, speedl, 0)
                        self.push_speed_right(t, speedr, 0)
                        self.push_pos_left(t, posl, 0)
                        self.push_pos_right(t, posr, 0)
                       # 1er argument le temps (time.time()), 2eme la valeur, 3eme la consigne.

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

        # Cas 1 : translation avec asserv en vitesse uniquement (2 roues allant à la meme vitesse)
        if speed is not None:   
            print("SPEED")
            speed_ticks = speed * coeff
            send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_SPEED, CAN_BOARD_ID_MOTOR, fmt_motor_set_speed.pack(int(speed_ticks), int(speed_ticks)))

        # Cas 1 : juste translation
        elif position is not None:
            print("POS")
            position_ticks = position * coeff
            send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_POS, CAN_BOARD_ID_MOTOR, fmt_motor_set_pos.pack(int(position_ticks), int(position_ticks)))

        # Cas 2 : juste rotation
        elif angle is not None:
            print("ANGLE")
            angle_ticks = angle * DISTANCE_BETWEEN_WHEELS / 2
            if angle >= 0:  # sens trigo
                send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_POS, CAN_BOARD_ID_MOTOR, fmt_motor_set_pos.pack(-int(angle_ticks), int(angle_ticks)))  # roue gauche, roue droite
            if angle < 0:
                send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_POS, CAN_BOARD_ID_MOTOR, fmt_motor_set_pos.pack(int(angle_ticks), -int(angle_ticks)))  # roue gauche, roue droite
        else:
            print("STOP")
            send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_STOP, CAN_BOARD_ID_MOTOR, [])

register_views(app, socketio, Adapter(socketio))

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)
