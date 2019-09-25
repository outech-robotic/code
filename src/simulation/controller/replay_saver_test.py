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
from src.simulation.entity.state import State, RobotID, Robot


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

    replay_saver.on_tick(
        State(
            time=0,
            robots={RobotID.RobotA: Robot(
                position=Vector2(0, 0),
                angle=0,
            )},
            cups=[],
        ))
    replay_saver.on_tick(
        State(
            time=1,
            robots={
                RobotID.RobotA: Robot(
                    position=Vector2(1, 2),
                    angle=3,
                ),
            },
            cups=[],
        ))

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
            'cups': [],
            'robots': {
                'ROBOT_A': {
                    'angle': 0,
                    'position': [0.0, 0.0]
                }
            },
            'time': 0
        }, {
            'cups': [],
            'robots': {
                'ROBOT_A': {
                    'angle': 3,
                    'position': [1.0, 2.0]
                }
            },
            'time': 1
        }]
    }
