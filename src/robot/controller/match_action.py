"""
Match action controller module.
"""
import asyncio


class MatchActionController:
    """Match action controller is a controller to use servos, pumps, valves."""

    def __init__(self):
        self.correction_position_wanted = False
        self.movement_done_event = asyncio.Event()

    async def raise_flag(self) -> None:
        """ Raise a flag with a servo. """

    async def retract_flag(self) -> None:
        """ Retract a flag with a servo. """

    async def raise_pump_arm(self) -> None:
        """ Prepare arm with pump to take a cup. """

    async def take_cup_with_pump(self) -> None:
        """ Take a cup : move 1 servo and activate pump. """

    async def put_down_cup_from_pump(self) -> None:
        """ Put down a cup : move 1 servo and deactivate pump. """

    async def raise_grabber_arm(self) -> None:
        """ Prepare arm with grabber to take a cup. """

    async def take_cup_with_grabber(self) -> None:
        """ Take a cup : move 3 servos + use pressure sensor. """

    async def put_down_cup_from_grabber(self) -> None:
        """ Put down a cup : move 3 servos + use pressure sensor. """

    async def open_door(self) -> None:
        """ Use a servo to open a door. """

    async def close_door(self) -> None:
        """ Use a servo to close a door. """

    async def deploy_windsock(self) -> None:
        """ Use a servo to open a windsock. """

    async def retract_windsock(self) -> None:
        """ Use a servo to close a windsock. """

    async def correct_position(self) -> None:
        """ Correct position with laser sensors. """
        self.correction_position_wanted = True
        self.movement_done_event.clear()
        await self.movement_done_event.wait()
        self.correction_position_wanted = False

    async def set_pressures(self) -> None:
        """ Get pressures from pressure sensors and determine if cup taken (true/false). """

    async def set_laser_distances(self) -> None:
        """ Get laser distances from laser sensors and correct position if wanted. """
