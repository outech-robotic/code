"""
RobotAdapter module.
"""
from src.robot.handler.motion import MotionHandler
from src.simulation.controller.subscriber import SimulationSubscriber
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.entity.state import State, RobotID


class RobotAdapter(SimulationSubscriber):
    """
    Take simulation state and trigger the handlers with the correct values. 
    This is the feedback loop of the simulation.
    """

    def __init__(
        self,
        simulation_configuration: SimulationConfiguration,
        motion_handler: MotionHandler,
    ):
        self.motion_handler = motion_handler
        self.simulation_configuration = simulation_configuration

    def on_tick(self, state: State) -> None:
        """
        Send location updates.
        """

        robot_a = state.robots[RobotID.RobotA]
        position = robot_a.position
        angle = robot_a.angle

        self.motion_handler.position_update(position.x, position.y, angle)
