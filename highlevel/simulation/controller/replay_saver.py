"""
Replay saver module.
"""
import json

from highlevel.logger import LOGGER
from highlevel.robot.entity.configuration import Configuration
from highlevel.simulation.client.http import HTTPClient
from highlevel.simulation.client.web_browser import WebBrowserClient
from highlevel.simulation.entity.simulation_configuration import SimulationConfiguration
from highlevel.util.json_encoder import RobotJSONEncoder
from highlevel.util.probe import Probe, DebugEvent

REPLAY_API_URL = 'https://replay-api.outech.fr/replay/'
REPLAY_VIEWER_URL = 'https://outech-robotic.github.io/replay/index.html'

# REPLAY_VIEWER_URL = 'http://127.0.0.1:8000/'


class ReplaySaver:
    """
    Save simulation state for future replay.
    """

    # pylint: disable=too-many-arguments
    def __init__(self, configuration: Configuration,
                 simulation_configuration: SimulationConfiguration,
                 http_client: HTTPClient, web_browser_client: WebBrowserClient,
                 probe: Probe):
        self.configuration = configuration
        self.simulation_configuration = simulation_configuration
        self.http_client = http_client
        self.web_browser_client = web_browser_client
        self.probe = probe

    def save_replay(self):
        """
        Save the replay.
        """
        frames, _ = self.probe.poll(
            rate=self.simulation_configuration.replay_fps)

        configuration_event = DebugEvent(
            key='configuration',
            time=frames[0].time,
            value=self.configuration,
        )
        simulation_configuration_event = DebugEvent(
            key='simulation_configuration',
            time=frames[0].time,
            value=self.simulation_configuration,
        )
        result = {
            'events':
            [configuration_event, simulation_configuration_event] + frames,
        }

        dump = json.dumps(result, cls=RobotJSONEncoder)
        LOGGER.get().info("saving_replay", size=len(dump))

        replay_id = self.http_client.post_file(REPLAY_API_URL, dump)['id']

        LOGGER.get().info("saved_replay", url=REPLAY_API_URL + replay_id)
        self.web_browser_client.open(REPLAY_VIEWER_URL + '?replay=' +
                                     REPLAY_API_URL + replay_id)
