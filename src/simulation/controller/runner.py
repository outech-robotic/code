"""
Simulation runner module.
"""
import asyncio

from src.robot.entity.configuration import Configuration
from src.simulation.controller.subscriber import SimulationSubscriber
from src.simulation.entity.event import EventQueue, EventType, EventOrder
from src.simulation.entity.simulation_configuration import SimulationConfiguration
from src.simulation.entity.state import State, Robot, RobotID
from src.simulation.gateway.simulation import SimulationGateway
from src.util.geometry.direction import forward


class SimulationRunner:
    """
    Run the simulation. Re-caculate the position of every object on the map at certain rate and 
    notify the robot by sending information to the sensors.
    """

    # Constructor with multiple dependencies:
    # pylint: disable=too-many-arguments,too-many-instance-attributes
    def __init__(self, event_queue: EventQueue,
                 simulation_gateway: SimulationGateway,
                 configuration: Configuration,
                 replay_saver: SimulationSubscriber,
                 robot_adapter: SimulationSubscriber,
                 simulation_configuration: SimulationConfiguration):

        self.event_queue = event_queue
        self.simulation_gateway = simulation_gateway
        self.replay_saver = replay_saver
        self.robot_adapter = robot_adapter
        self.simulation_configuration = simulation_configuration

        self.tick = 0

        self.state = State(
            time=0,
            robots={
                RobotID.RobotA:
                    Robot(
                        position=configuration.initial_position,
                        angle=configuration.initial_angle,
                    )
            },
            cups=[],
        )
        self.running = True

    async def run(self) -> None:
        """
        Run the simulation.
        """
        while self.running:
            current_tick = self.tick
            events = self.event_queue.pop(current_tick)

            # Process all the events.
            for event in events:
                self._process_event(event.event)

            # Send the feedback to the subscribers.
            self._notify_subscribers()

            self.tick = current_tick + 1
            self.state.time = int(self.tick /
                                  self.simulation_configuration.tickrate * 1000)
            await asyncio.sleep(1 / self.simulation_configuration.tickrate /
                                self.simulation_configuration.speed_factor)

    def stop(self):
        """
        Stop the simulation from running.
        """
        self.running = False

    def _process_event(self, event: EventOrder) -> None:
        """
        Process an event.
        """

        if event.type == EventType.MOVE_FORWARD:
            robot = self.state.robots[event.payload['robot_id']]
            robot.position += forward(robot.angle) * event.payload['distance']

        elif event.type == EventType.ROTATE:
            robot = self.state.robots[event.payload['robot_id']]
            robot.angle += event.payload['angle']

        elif event.type == EventType.MOVEMENT_DONE:
            self.simulation_gateway.movement_done()

        else:
            raise RuntimeError(f"cannot handle event {event}")

    def _notify_subscribers(self) -> None:
        """
        Notify the subscribers of state change.
        """
        self.robot_adapter.on_tick(self.state.clone())
        self.replay_saver.on_tick(self.state.clone())
