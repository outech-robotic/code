"""
Simulation gateway module.
"""
from src.robot.handler.motion import MotionHandler
from src.simulation.entity.simulation_configuration import SimulationConfiguration


class SimulationGateway:
    """
    This is the gateway from the simulation world to the "real" world. 
    This gateway communicates with the sensors of the robot (and thus the handlers of the robot).
    """

    def __init__(self, motion_handler: MotionHandler,
                 simulation_configuration: SimulationConfiguration):
        self.simulation_configuration = simulation_configuration
        self.motion_handler = motion_handler

    def movement_done(self) -> None:
        """
        Send the "movement done" signal to the robot.
        """
        self.motion_handler.movement_done()
