import random
import string
from math import sin
from threading import Thread
from time import sleep, time
from typing import Optional

from flask import Flask
from flask_socketio import SocketIO

from views import register_views, InterfaceAdapter, PID

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
        print(f"Got PID {speed_left} {speed_right} {pos_left} {pos_right}")

    def on_order_submission(self, speed: Optional[float], position: Optional[float],
                            angle: Optional[float]):
        # Ici tu implem l'envoi des paquets sur le CAN.
        print(f"Got order {speed} {position} {angle}")


register_views(app, socketio, Adapter(socketio))

if __name__ == '__main__':
    socketio.run(app)
