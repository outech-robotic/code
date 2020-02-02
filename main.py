import random
import string
import struct
from math import pi
from threading import Thread
from time import time
from typing import Optional

import can
from flask import Flask
from flask_socketio import SocketIO

from views import register_views, InterfaceAdapter, PID

CAN_BOARD_ID_WIDTH = 5
CAN_MSG_WIDTH = 9
CAN_BOARD_ID_MOTOR = 15
CAN_CHANNEL_MOTOR = 0b00
CAN_MSG_SPEED = 0b1000    # orders the movement of both wheels at constant speed (2x32bits signed, left and right encoder speeds)
CAN_MSG_POS = 0b0010      # orders a movement of both wheels (data: 2x32bits signed, left and right encoder position)
CAN_MSG_STOP = 0b0000     # Stops robot on the spot, resetting all errors of PIDs
CAN_MSG_MCS_MODE = 0b1001 # Sets motion control mode : data: 1 byte: bit 0 = speed mode on/off, bit 1 = position mode on/off
CAN_KP_ID = 0b1100        # sets proportionnal constant of PID of ID at first byte of data, value is the following 32 bits (unsigned, 65535 * value in floating point of KP)
CAN_KI_ID = 0b1101        # same for integral constant
CAN_KD_ID = 0b1110        # and derivative

MCS_MODE_SPEED = 0b01
MCS_MODE_POS   = 0b10
MCS_MODE_BOTH  = 0b11

# PID IDs
PID_LEFT_SPEED = 0
PID_LEFT_POS = 1
PID_RIGHT_SPEED = 2
PID_RIGHT_POS = 3

COD_UPDATE_FREQ = 100  # Hz, encoder positions sent from Motion control board each seconds

# Formatters for data packing
fmt_motor_set_speed = struct.Struct('<ll')  # 32b + 32b signe
fmt_motor_set_pos = struct.Struct('<ll')
fmt_motor_cod_pos = struct.Struct('<ll')
fmt_motor_set_pid = struct.Struct('<BL')  # 8b + 32b non signes

#Physical constants of the robot
WHEEL_DIAMETER = 70.0  # en mm ; 2400 ticks par tour donc par 2*pi*60 mm
DISTANCE_BETWEEN_WHEELS = 365.0  # en mm
TICKS_PER_TURN = 2400.0
MM_TO_TICK = TICKS_PER_TURN / (pi * WHEEL_DIAMETER)
TICK_TO_MM = (pi*WHEEL_DIAMETER)/TICKS_PER_TURN

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
        self.setpoint_speed = 0.0
        self.setpoint_pos   = None
        self.setpoint_angle = None
        # A la place de cette fonction et du thread, on met le code qui recoit les msg CAN et on
        # appelle les fonctions .push_*_*
        def f():
            start_t = time()
            last_left, last_right = 0, 0
            with can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000) as bus:
                for message in bus:
                    if (message.arbitration_id >> 5) == 0b000011:
                        # sleep(1.0/COD_UPDATE_FREQ)
                        posl, posr = fmt_motor_cod_pos.unpack(message.data)
                        # print(posl, posr)
                        speedl, speedr = (posl - last_left) * COD_UPDATE_FREQ, (
                                    posr - last_right) * COD_UPDATE_FREQ
                        # print(speedl, speedr)
                        last_left, last_right = posl, posr
                        t = int((time() - start_t) * 1000)
                        self.push_speed_left(t, speedl, self.setpoint_speed)
                        self.push_speed_right(t, speedr, self.setpoint_speed)
                        setpoint_left, setpoint_right = 0.0, 0.0
                        if self.setpoint_pos is not None:
                            setpoint_left = self.setpoint_pos
                            setpoint_right = self.setpoint_pos
                        elif self.setpoint_angle is not None:
                            setpoint_left = -self.setpoint_angle
                            setpoint_right = self.setpoint_angle
                        self.push_pos_left(t, posl, setpoint_left)
                        self.push_pos_right(t, posr, setpoint_right)
                    # 1er argument le temps (time.time()), 2eme la valeur, 3eme la consigne.

        Thread(target=f).start()

    def on_pid_submission(self, speed_left: PID, speed_right: PID, pos_left: PID,
                          pos_right: PID) -> None:
        # Ici tu implem l'envoi des paquets sur le CAN.
        print(f"Got PID {pos_left} {pos_right} {speed_left} {speed_right}")

        pos_left.p = round(pos_left.p * 65535)
        pos_left.i = round(pos_left.i * 65535)
        pos_left.d = round(pos_left.d * 65535)

        pos_right.p = round(pos_right.p * 65535)
        pos_right.i = round(pos_right.i * 65535)
        pos_right.d = round(pos_right.d * 65535)

        speed_left.p = round(speed_left.p * 65535)
        speed_left.i = round(speed_left.i * 65535)
        speed_left.d = round(speed_left.d * 65535)

        speed_right.p = round(speed_right.p * 65535)
        speed_right.i = round(speed_right.i * 65535)
        speed_right.d = round(speed_right.d * 65535)

        send_packet(CAN_CHANNEL_MOTOR, CAN_KP_ID, CAN_BOARD_ID_MOTOR,
                    fmt_motor_set_pid.pack(PID_LEFT_POS, int(pos_left.p)))
        send_packet(CAN_CHANNEL_MOTOR, CAN_KI_ID, CAN_BOARD_ID_MOTOR,
                    fmt_motor_set_pid.pack(PID_LEFT_POS, int(pos_left.i)))
        send_packet(CAN_CHANNEL_MOTOR, CAN_KD_ID, CAN_BOARD_ID_MOTOR,
                    fmt_motor_set_pid.pack(PID_LEFT_POS, int(pos_left.d)))

        send_packet(CAN_CHANNEL_MOTOR, CAN_KP_ID, CAN_BOARD_ID_MOTOR,
                    fmt_motor_set_pid.pack(PID_RIGHT_POS, int(pos_right.p)))
        send_packet(CAN_CHANNEL_MOTOR, CAN_KI_ID, CAN_BOARD_ID_MOTOR,
                    fmt_motor_set_pid.pack(PID_RIGHT_POS, int(pos_right.i)))
        send_packet(CAN_CHANNEL_MOTOR, CAN_KD_ID, CAN_BOARD_ID_MOTOR,
                    fmt_motor_set_pid.pack(PID_RIGHT_POS, int(pos_right.d)))

        send_packet(CAN_CHANNEL_MOTOR, CAN_KP_ID, CAN_BOARD_ID_MOTOR,
                    fmt_motor_set_pid.pack(PID_LEFT_SPEED, int(speed_left.p)))
        send_packet(CAN_CHANNEL_MOTOR, CAN_KI_ID, CAN_BOARD_ID_MOTOR,
                    fmt_motor_set_pid.pack(PID_LEFT_SPEED, int(speed_left.i)))
        send_packet(CAN_CHANNEL_MOTOR, CAN_KD_ID, CAN_BOARD_ID_MOTOR,
                    fmt_motor_set_pid.pack(PID_LEFT_SPEED, int(speed_left.d)))

        send_packet(CAN_CHANNEL_MOTOR, CAN_KP_ID, CAN_BOARD_ID_MOTOR,
                    fmt_motor_set_pid.pack(PID_RIGHT_SPEED, int(speed_right.p)))
        send_packet(CAN_CHANNEL_MOTOR, CAN_KI_ID, CAN_BOARD_ID_MOTOR,
                    fmt_motor_set_pid.pack(PID_RIGHT_SPEED, int(speed_right.i)))
        send_packet(CAN_CHANNEL_MOTOR, CAN_KD_ID, CAN_BOARD_ID_MOTOR,
                    fmt_motor_set_pid.pack(PID_RIGHT_SPEED, int(speed_right.d)))

    def on_order_submission(self, speed: Optional[float], position: Optional[float],
                            angle: Optional[float]):
        # Ici tu implem l'envoi des paquets sur le CAN.
        print(
            f"Got order {speed} {position} {angle}")  # position en mm, speed en mm/s, angle en degré
       
        self.setpoint_speed = 0.0
        self.setpoint_pos   = None
        self.setpoint_angle = None
        # Cas 1 : translation avec asserv en vitesse uniquement (2 roues allant à la meme vitesse)
        if speed is not None:
            send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_MCS_MODE, CAN_BOARD_ID_MOTOR, MCS_MODE_SPEED)
            print("SPEED")
            self.setpoint_speed = speed
            speed_ticks = speed * MM_TO_TICK
            send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_SPEED, CAN_BOARD_ID_MOTOR,
                        fmt_motor_set_speed.pack(int(speed_ticks), int(speed_ticks)))

        # Cas 2 : juste translation, les vitesses des roues sont gérées par le LL
        elif position is not None:
            send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_MCS_MODE, CAN_BOARD_ID_MOTOR, MCS_MODE_BOTH)
            print("POS")
            self.setpoint_pos = position # position in mm that each wheel have to travel
            position_ticks = position * MM_TO_TICK # in ticks for Motion control board
            send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_POS, CAN_BOARD_ID_MOTOR,
                        fmt_motor_set_pos.pack(int(position_ticks), int(position_ticks)))

        # Cas 3 : juste rotation
        elif angle is not None:
            send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_MCS_MODE, CAN_BOARD_ID_MOTOR, MCS_MODE_BOTH)
            print("ANGLE")
            # distance for each wheel(in opposite direcitons), to reach angle, in mm for graph
            self.setpoint_angle = angle * DISTANCE_BETWEEN_WHEELS / 2 
            angle_ticks = self.setpoint_angle*MM_TO_TICK # in ticks for Motion control board
            
            if angle >= 0:  # sens trigo
                send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_POS, CAN_BOARD_ID_MOTOR,
                            fmt_motor_set_pos.pack(-int(angle_ticks),
                                                   int(angle_ticks)))  # roue gauche, roue droite
            if angle < 0:
                send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_POS, CAN_BOARD_ID_MOTOR,
                            fmt_motor_set_pos.pack(int(angle_ticks),
                                                   -int(angle_ticks)))  # roue gauche, roue droite
        else:
            print("STOP")
            send_packet(CAN_CHANNEL_MOTOR, CAN_MSG_STOP, CAN_BOARD_ID_MOTOR, [])

    def on_stop_button(self):
        self.on_order_submission(0, 0, 0)


register_views(app, socketio, Adapter(socketio))

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000)
