"""
Replay saver module.
"""
import json
from dataclasses import dataclass, asdict
from typing import List, Tuple

import structlog

from src.robot.entity.configuration import Configuration
from src.simulation.client.http import HTTPClient
from src.simulation.client.web_browser import WebBrowserClient
from src.simulation.controller.subscriber import SimulationSubscriber
from src.simulation.entity.simulation_configuration import SimulationConfiguration

LOGGER = structlog.get_logger()

REPLAY_API_URL = 'https://replay-api.outech.fr/replay/'
REPLAY_VIEWER_URL = 'https://nicolasbon.net/replay/'


@dataclass
class Position:
    """
    Position of a given entity on the map.
    Sub-part of the JSON.
    """
    type: str
    x: int
    y: int
    angle: float


@dataclass
class Frame:
    """
    Simulation frame.
    Sub-part of the JSON.
    """
    time: int
    positions: List[Position]


@dataclass
class Replay:
    """
    Replay.
    Sub-part of the JSON.
    """
    robot_size: Tuple[int, int]
    frames: List[Frame]


class ReplaySaver(SimulationSubscriber):
    """
    Save simulation state for future replay.
    """

    def __init__(self, configuration: Configuration,
                 simulation_configuration: SimulationConfiguration,
                 http_client: HTTPClient, web_browser_client: WebBrowserClient):
        self.result = Replay(robot_size=(int(configuration.robot_length),
                                         int(configuration.robot_width)),
                             frames=[])
        self.configuration = configuration
        self.simulation_configuration = simulation_configuration
        self.http_client = http_client
        self.web_browser_client = web_browser_client

    def on_tick(self, state: dict) -> None:
        robot_pos = state['robot']['position']
        robot_ang = state['robot']['angle']
        time = (state['tick'] / self.simulation_configuration.tickrate) * 1000
        self.result.frames.append(
            Frame(time=int(time),
                  positions=[
                      Position(type='robot',
                               x=int(robot_pos[0]),
                               y=int(robot_pos[1]),
                               angle=float(robot_ang))
                  ]))

    def save_replay(self):
        """
        Save the replay.
        """

        res = asdict(self.result)
        dump = json.dumps(res)
        LOGGER.info("saving_replay", size=len(dump))

        replay_id = self.http_client.post_file(REPLAY_API_URL, dump)['id']

        LOGGER.info("saved_replay", url=REPLAY_API_URL + replay_id)
        self.web_browser_client.open(REPLAY_VIEWER_URL + '?replay=' +
                                     REPLAY_API_URL + replay_id)
