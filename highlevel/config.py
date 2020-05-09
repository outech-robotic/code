"""
Contains configuration.
"""
from highlevel.robot.entity.color import Color
from highlevel.robot.entity.configuration import Configuration
from highlevel.robot.entity.configuration import DebugConfiguration
from highlevel.simulation.entity.simulation_configuration import SimulationConfiguration
from highlevel.util.geometry.segment import Segment
from highlevel.util.geometry.vector import Vector2

CONFIG = Configuration(
    initial_position=Vector2(200, 1200),
    initial_angle=0,
    robot_width=380,
    robot_length=240,
    field_shape=(3000, 2000),
    color=Color.BLUE,
    wheel_radius=73.8 / 2,
    encoder_ticks_per_revolution=2400,
    distance_between_wheels=357,
    debug=DebugConfiguration(port=8080, host="0.0.0.0", refresh_rate=10),
)

SIMULATION_CONFIG = SimulationConfiguration(
    speed_factor=1e100,  # Run the simulation as fast as possible.
    obstacles=[
        Segment(start=Vector2(0, 0), end=Vector2(0, CONFIG.field_shape[1])),
        Segment(start=Vector2(0, 0), end=Vector2(CONFIG.field_shape[0], 0)),
        Segment(start=Vector2(*CONFIG.field_shape),
                end=Vector2(0, CONFIG.field_shape[1])),
        Segment(start=Vector2(*CONFIG.field_shape),
                end=Vector2(CONFIG.field_shape[0], 0)),
    ])
