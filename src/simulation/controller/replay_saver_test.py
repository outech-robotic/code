"""
Test for replay saver module.
"""
import json
from unittest.mock import MagicMock

from src.simulation.client.http import HTTPClient
from src.simulation.client.web_browser import WebBrowserClient
from src.simulation.controller.replay_saver import ReplaySaver
from src.simulation.entity.simulation_configuration import SimulationConfiguration


def test_happy_path(configuration_test):
    """
    Happy path.
    """

    http_client = MagicMock(spec=HTTPClient)
    http_client.post_file = MagicMock(return_value={'id': 'test_id'})
    replay_saver = ReplaySaver(
        configuration=configuration_test,
        simulation_configuration=SimulationConfiguration(obstacles=[]),
        http_client=http_client,
        web_browser_client=MagicMock(spec=WebBrowserClient))

    replay_saver.on_tick({
        "time": 0,
        "robots": {
            "ROBOT_A": {
                "angle": 0.0,
                "position": {
                    "x": 0,
                    "y": 0,
                }
            }
        }
    })
    replay_saver.on_tick({
        "time": 1,
        "robots": {
            "ROBOT_A": {
                "angle": 3,
                "position": {
                    "x": 1,
                    "y": 2,
                }
            }
        }
    })

    replay_saver.save_replay()

    http_client.post_file.assert_called_once()

    # Make sure the 2nd argument of the call is a valid JSON string and that it conforms to the
    # JSON expected output.
    data = http_client.post_file.call_args_list[0][0][1]
    assert json.loads(data) == {
        'initial_configuration': {
            'sizes': {
                'ROBOT_A': [10, 10]
            }
        },
        'frames': [{
            "time": 0,
            "robots": {
                "ROBOT_A": {
                    "angle": 0.0,
                    "position": {
                        "x": 0,
                        "y": 0,
                    }
                }
            }
        }, {
            "time": 1,
            "robots": {
                "ROBOT_A": {
                    "angle": 3,
                    "position": {
                        "x": 1,
                        "y": 2,
                    }
                }
            }
        }]
    }
