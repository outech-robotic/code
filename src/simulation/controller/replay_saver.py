"""
Replay saver module.
"""
import io
import json
import time
import webbrowser
from dataclasses import dataclass, asdict
from typing import List

import requests
import structlog

from src.simulation.controller.simulation_subscriber import SimulationSubscriber

LOGGER = structlog.get_logger()

REPLAY_API_URL = 'https://replay-api.outech.fr/replay/'
REPLAY_VIEWER_URL = 'https://nicolasbon.net/replay/'


@dataclass
class Position:
    type: str
    x: int
    y: int
    angle: float


@dataclass
class Frame:
    time: int
    positions: List[Position]


@dataclass
class Replay:
    frames: List[Frame]


class ReplaySaver(SimulationSubscriber):
    """
    Save simulation state for future replay.
    """

    def __init__(self):
        self.result = Replay(frames=[],)
        self.start_time = time.perf_counter()

    def on_tick(self, state: dict) -> None:
        robot_pos = state.get('robot').get('position')
        robot_ang = state.get('robot').get('angle')
        self.result.frames.append(
            Frame(time=int((time.perf_counter() - self.start_time) * 1000),
                  positions=[
                      Position(type='robot',
                               x=int(robot_pos[0]),
                               y=int(robot_pos[1]),
                               angle=robot_ang)
                  ]))

    def save_replay(self):
        """
        Save the replay.
        """

        res = asdict(self.result)
        dump = json.dumps(res)
        LOGGER.info("saving_replay", size=len(dump))

        f = io.StringIO(dump)
        files = {'my_file': f}

        r = requests.post(REPLAY_API_URL, files=files)
        replay_id = r.json().get('id')

        LOGGER.info("saved_replay", url=REPLAY_API_URL + replay_id)
        webbrowser.open(REPLAY_VIEWER_URL + '?replay=' + REPLAY_API_URL +
                        replay_id)
