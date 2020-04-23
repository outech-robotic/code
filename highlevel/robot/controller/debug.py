"""
Debug module for live debugging.
"""
import asyncio
import json

import websockets
from websockets.http import Headers
from websockets.protocol import State

from highlevel.logger import LOGGER
from highlevel.robot.entity.configuration import Configuration
from highlevel.simulation.controller.probe import SimulationProbe
from highlevel.util.json_encoder import RobotJSONEncoder


class DebugController:
    """
    Class that sends periodically the state of the robot on a websocket.
    """
    def __init__(self, configuration: Configuration,
                 simulation_probe: SimulationProbe,
                 event_loop: asyncio.AbstractEventLoop):
        self._configuration = configuration
        self._simulation_probe = simulation_probe
        self._event_loop = event_loop

    async def _callback(self, websocket: websockets.WebSocketServerProtocol,
                        _: str) -> None:
        """
        Function executed for each new incoming connection.
        """
        LOGGER.get().info("new_debug_connection")
        while websocket.state in (State.CONNECTING, State.OPEN):
            data = self._simulation_probe.probe()
            json_data = json.dumps(data, cls=RobotJSONEncoder)
            await websocket.send(json_data)
            await asyncio.sleep(1 / self._configuration.debug.refresh_rate)

    async def run(self) -> None:
        """
        Run the debug server.
        """
        LOGGER.get().info("websocket_debug_serve", port=self._configuration.debug.port)

        async with websockets.serve(self._callback,
                                    host=self._configuration.debug.host,
                                    port=self._configuration.debug.port,
                                    loop=self._event_loop,
                                    process_request=process_request) as server:
            await server.wait_closed()


async def process_request(_: str, request_headers: Headers) -> None:
    """
    Enable CORS.
    """
    request_headers['Access-Control-Allow-Origin'] = '*'
