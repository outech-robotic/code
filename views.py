import json
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

from flask import request, redirect, render_template, Flask
from flask_socketio import SocketIO

from forms import AllPIDForms, OrderForm

FILE_NAME = 'pid_coef.json'


@dataclass
class PID:
    p: float
    i: float
    d: float


def to_pid(tbl: dict) -> PID:
    return PID(
        p=tbl['P'],
        i=tbl['I'],
        d=tbl['D'],
    )


class InterfaceAdapter(ABC):
    def __init__(self, socketio: SocketIO):
        self._socketio = socketio

    def push_pos_right(self, t, v, setpoint):
        self._socketio.emit('pos_right', [t, v, setpoint], broadcast=True)

    def push_pos_left(self, t, v, setpoint):
        self._socketio.emit('pos_left', [t, v, setpoint], broadcast=True)

    def push_speed_right(self, t, v, setpoint):
        self._socketio.emit('speed_right', [t, v, setpoint], broadcast=True)

    def push_speed_left(self, t, v, setpoint):
        self._socketio.emit('speed_left', [t, v, setpoint], broadcast=True)

    @abstractmethod
    def on_pid_submission(self,
                          speed_left: PID, speed_right: PID,
                          pos_left: PID, pos_right: PID) -> None:
        """ PID submission. """

    @abstractmethod
    def on_order_submission(self,
                            speed: Optional[float],
                            position: Optional[float],
                            angle: Optional[float]):
        """ Order submission. """

    @abstractmethod
    def on_stop_button(self):
        """ On stop button pressed."""


def get_pid_coefs():
    try:
        with open(FILE_NAME, "r") as f:
            return json.loads(f.read())
    except FileNotFoundError:
        return {
            'PosLeft': {'P': 1.0, 'I': 2.0, 'D': 3.0, },
            'PosRight': {'P': 4.0, 'I': 5.0, 'D': 6.0, },
            'SpeedLeft': {'P': 7.0, 'I': 8.0, 'D': 9.0, },
            'SpeedRight': {'P': 10.0, 'I': 11.0, 'D': 12.0, },
        }


def get_saved_pid_form():
    form = AllPIDForms()
    for subform, data in get_pid_coefs().items():
        if isinstance(data, dict):
            for field, value in data.items():
                form[subform][field].data = value
    return form


def register_views(app: Flask, socketio: SocketIO, interface: InterfaceAdapter):
    @app.route('/pid', methods=['GET', 'POST'])
    def pid_view():
        if request.method == 'GET':
            return redirect('/')

        form = AllPIDForms()
        if form.validate_on_submit():
            with open(FILE_NAME, "w") as f:
                f.write(json.dumps(form.data))

            interface.on_pid_submission(
                speed_left=to_pid(form.data['SpeedLeft']),
                speed_right=to_pid(form.data['SpeedRight']),
                pos_right=to_pid(form.data['PosRight']),
                pos_left=to_pid(form.data['PosLeft']),
            )

        return render_template(
            'index.html',
            form=form,
            order_form=OrderForm(),
            capture=False,
        )

    @app.route('/order', methods=['GET', 'POST'])
    def order_view():
        if request.method == 'GET':
            return redirect('/')

        form = OrderForm()
        if form.validate_on_submit():
            interface.on_order_submission(
                speed=form.data['Speed'],
                position=form.data['Position'],
                angle=form.data['Angle'],
            )

        return render_template(
            'index.html',
            form=get_saved_pid_form(),
            order_form=form,
            capture=True,
        )

    @app.route('/')
    def index():
        return render_template(
            'index.html',
            form=get_saved_pid_form(),
            order_form=OrderForm(),
            capture=False,
        )

    @socketio.on('STOP')
    def on_stop(_):
        interface.on_stop_button()
