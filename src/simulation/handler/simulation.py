"""
Simulation handler module.
"""
from src.robot.entity.type import Radian, Millimeter
from src.robot.gateway.motion import MotionGateway
from src.simulation.controller.simulation_controller import SimulationController


class SimulationHandler(MotionGateway):
    """
    Intercept all the "real-world" orders (i.e. move forward) and convert them into actions inside 
    the simulation. This is the entry point of the simulation (and thus is also the exit point of 
    the robot).
    """

    def __init__(self, simulation_controller: SimulationController):
        self.simulation_controller = simulation_controller

    def move_forward(self, distance: Millimeter) -> None:
        """
        Move the robot forward.
        """
        self.simulation_controller.robot_move_forward(distance)

    def rotate(self, angle: Radian) -> None:
        """
        Rotate the robot.
        """
        self.simulation_controller.robot_rotate(angle)
