"""
Replay saver module.
"""
import json
from dataclasses import asdict

import structlog

from src.robot.entity.configuration import Configuration
from src.simulation.client.http import HTTPClient
from src.simulation.client.web_browser import WebBrowserClient
from src.simulation.controller.subscriber import SimulationSubscriber
from src.simulation.entity.replay import InitialConfiguration, Replay
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.entity.state import RobotID, State
from src.util.json_encoder import RobotJSONEncoder

LOGGER = structlog.get_logger()

REPLAY_API_URL = 'https://replay-api.outech.fr/replay/'
REPLAY_VIEWER_URL = 'https://nicolasbon.net/replay/'


class ReplaySaver(SimulationSubscriber):
    """
    Save simulation state for future replay.
    """

    def __init__(self, configuration: Configuration,
                 simulation_configuration: SimulationConfiguration,
                 http_client: HTTPClient, web_browser_client: WebBrowserClient):
        size = (int(configuration.robot_length), int(configuration.robot_width))
        initial_configuration = InitialConfiguration(sizes={
            RobotID.RobotA: size,
        })

        self.result = Replay(initial_configuration=initial_configuration,
                             frames=tuple())
        self.configuration = configuration
        self.simulation_configuration = simulation_configuration
        self.http_client = http_client
        self.web_browser_client = web_browser_client

    def on_tick(self, state: State) -> None:
        self.result = self.result.add_frame(state)

    def save_replay(self):
        """
        Save the replay.
        """

        res = asdict(self.result)
        dump = json.dumps(res, cls=RobotJSONEncoder)
        LOGGER.info("saving_replay", size=len(dump))

        replay_id = self.http_client.post_file(REPLAY_API_URL, dump)['id']

        LOGGER.info("saved_replay", url=REPLAY_API_URL + replay_id)
        self.web_browser_client.open(REPLAY_VIEWER_URL + '?replay=' +
                                     REPLAY_API_URL + replay_id)
