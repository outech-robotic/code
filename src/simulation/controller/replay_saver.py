"""
Replay saver module.
"""
import json
from typing import List

from src.logger import LOGGER
from src.robot.entity.motion.configuration import Configuration
from src.simulation.client.http import HTTPClient
from src.simulation.client.web_browser import WebBrowserClient
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.entity.simulation_state import RobotID

REPLAY_API_URL = 'https://replay-api.outech.fr/replay/'
# REPLAY_VIEWER_URL = 'https://nicolasbon.net/replay/'
REPLAY_VIEWER_URL = 'http://127.0.0.1:8000/'


class ReplaySaver:
    """
    Save simulation state for future replay.
    """

    def __init__(self, configuration: Configuration,
                 simulation_configuration: SimulationConfiguration,
                 http_client: HTTPClient, web_browser_client: WebBrowserClient):
        size = (int(configuration.robot_length), int(configuration.robot_width))

        self.frames: List[dict] = []
        self.result = {
            'initial_configuration': {
                'sizes': {
                    RobotID.RobotA: size,
                }
            },
            'frames': self.frames,
        }
        self.configuration = configuration
        self.simulation_configuration = simulation_configuration
        self.http_client = http_client
        self.web_browser_client = web_browser_client

    def on_tick(self, state: dict) -> None:
        """
        Should be called to append the state to the result to be saved.
        """
        self.frames.append(state)

    def save_replay(self):
        """
        Save the replay.
        """

        dump = json.dumps(self.result)
        LOGGER.get().info("saving_replay", size=len(dump))

        replay_id = self.http_client.post_file(REPLAY_API_URL, dump)['id']

        LOGGER.get().info("saved_replay", url=REPLAY_API_URL + replay_id)
        self.web_browser_client.open(REPLAY_VIEWER_URL + '?replay=' +
                                     REPLAY_API_URL + replay_id)
