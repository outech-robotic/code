"""
Test for replay saver module.
"""
import json
from unittest.mock import MagicMock

from src.robot.entity.configuration import Configuration
from src.robot.entity.vector import Vector2
from src.simulation.client.http import HTTPClient
from src.simulation.client.web_browser import WebBrowserClient
from src.simulation.controller.replay_saver import ReplaySaver
from src.simulation.entity.simulation_configuration import SimulationConfiguration


def test_happy_path():
    """
    Happy path.
    """

    http_client = MagicMock(spec=HTTPClient)
    http_client.post_file = MagicMock(return_value={'id': 'test_id'})
    replay_saver = ReplaySaver(
        configuration=Configuration(
            initial_position=Vector2(0, 0),
            initial_angle=0,
            robot_width=10,
            robot_length=10,
            field_shape=(100, 100),
        ),
        simulation_configuration=SimulationConfiguration(obstacles=[]),
        http_client=http_client,
        web_browser_client=MagicMock(spec=WebBrowserClient))

    replay_saver.on_tick({
        'tick': 1,
        'robot': {
            'position': (0, 0),
            'angle': 0,
        }
    })
    replay_saver.on_tick({
        'tick': 2,
        'robot': {
            'position': (1, 2),
            'angle': 3,
        }
    })

    replay_saver.save_replay()

    http_client.post_file.assert_called_once()

    # Make sure the 2nd argument of the call is a valid JSON string and that it conforms to the
    # JSON expected output.
    data = http_client.post_file.call_args_list[0][0][1]
    assert json.loads(data) == {
        "robot_size": [10, 10],
        "frames": [{
            "time": 33,
            "positions": [{
                "type": "robot",
                "x": 0,
                "y": 0,
                "angle": 0.0
            }]
        }, {
            "time": 66,
            "positions": [{
                "type": "robot",
                "x": 1,
                "y": 2,
                "angle": 3.0
            }]
        }]
    }
