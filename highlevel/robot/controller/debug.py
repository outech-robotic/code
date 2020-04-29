"""
Debug module for live debugging.
"""
import asyncio
import inspect
import json
from dataclasses import asdict
from typing import List, Callable, Awaitable, Dict, Any

import websockets
from aiohttp import web
from websockets.http import Headers
from websockets.protocol import State

from highlevel.logger import LOGGER
from highlevel.robot.controller.motion.localization import LocalizationController
from highlevel.robot.entity.configuration import Configuration
from highlevel.robot.gateway.motion import MotionGateway
from highlevel.simulation.entity.simulation_configuration import SimulationConfiguration
from highlevel.util.get_methods import get_methods
from highlevel.util.json_encoder import RobotJSONEncoder
from highlevel.util.probe import Probe, DebugEvent


def get_last_args_sent() -> Dict[str, Dict[str, Any]]:
    try:
        with open("debug_last_arguments.json", "r") as f:
            data = json.loads(f.read())

        return data
    except FileNotFoundError:
        return {}


def set_last_args_sent(last_args: Dict[str, Dict[str, Any]]) -> None:
    with open("debug_last_arguments.json", "w") as f:
        f.write(json.dumps(last_args))


class DebugController:
    """
    Class that sends periodically the state of the robot on a websocket.
    """
    def __init__(self, configuration: Configuration,
                 simulation_configuration: SimulationConfiguration,
                 probe: Probe, event_loop: asyncio.AbstractEventLoop,
                 motion_gateway: MotionGateway, localization_controller: LocalizationController):
        self._configuration = configuration
        self._simulation_configuration = simulation_configuration
        self._probe = probe
        self._event_loop = event_loop
        self._actions = get_methods(motion_gateway) + get_methods(localization_controller)

    async def run(self) -> None:
        host = self._configuration.debug.host
        stop_http = await self.start_http_server(host, self._configuration.debug.http_port)
        try:
            await self.run_websocket_server(host, self._configuration.debug.websocket_port)
        finally:
            await stop_http()

    async def start_http_server(self, host: str, port: int) -> Callable[[], Awaitable[None]]:
        LOGGER.get().info("http_server_start", port=port)

        app = web.Application()
        app.add_routes([
            web.get('/action', self._http_get_actions),
            web.post('/action', self._http_post_action),
        ])

        async def on_prepare(_, response):
            response.headers['Access-Control-Allow-Origin'] = '*'

        app.on_response_prepare.append(on_prepare)

        runner = web.AppRunner(app)
        await runner.setup()

        site = web.TCPSite(runner, host, port)
        await site.start()

        async def stop():
            await runner.cleanup()

        return stop

    async def run_websocket_server(self, host: str, port: int) -> None:
        """
        Run the debug server.
        """

        LOGGER.get().info("websocket_debug_serve", port=port)
        async with websockets.serve(self._websocket_callback,
                                    host=host,
                                    port=port,
                                    loop=self._event_loop,
                                    process_request=process_request) as server:
            await server.wait_closed()

    async def _http_post_action(self, request):
        data = await request.json()
        if 'name' not in data or 'args' not in data:
            return web.HTTPBadRequest()

        function_name = data['name']
        function_args = data['args']
        if function_name not in map(lambda a: a.name, self._actions):
            return web.HTTPNotFound()

        last_args = get_last_args_sent()
        last_args[function_name] = function_args
        set_last_args_sent(last_args)

        for action in self._actions:
            if action.name != function_name:
                continue

            result = action.func(**function_args)
            if inspect.isawaitable(result):
                await result

        return web.HTTPNoContent()

    async def _http_get_actions(self, _):
        response = {
            'functions': []
        }
        for action in self._actions:
            response['functions'].append({
                'name': action.name,
                'args': action.args,
                'documentation': action.documentation,
                'last_args_sent': get_last_args_sent().get(action.name, {})
            })
        return web.json_response(response)

    def _get_config_events(self, time: float) -> List[DebugEvent]:
        configuration_event = DebugEvent(
            key='configuration',
            time=time,
            value=asdict(self._configuration),
        )
        simulation_configuration_event = DebugEvent(
            key='simulation_configuration',
            time=time,
            value=asdict(self._simulation_configuration),
        )
        return [configuration_event, simulation_configuration_event]

    async def _websocket_callback(self, websocket: websockets.WebSocketServerProtocol,
                                  _: str) -> None:
        """
        Function executed for each new incoming connection.
        """
        LOGGER.get().info("new_debug_connection")

        LOGGER.get().debug("sending_config_to_debug_client")
        data, cursor = self._probe.poll()
        config_events = self._get_config_events(data[-1].time)

        config_json_data = json.dumps(config_events, cls=RobotJSONEncoder)
        await websocket.send(config_json_data)

        LOGGER.get().debug("sending_live_events_to_debug_client")
        while websocket.state in (State.CONNECTING, State.OPEN):
            data, cursor = self._probe.poll(
                cursor=cursor, rate=self._configuration.debug.refresh_rate)
            json_data = json.dumps(data, cls=RobotJSONEncoder)
            await websocket.send(json_data)
            await asyncio.sleep(1 / self._configuration.debug.refresh_rate)


async def process_request(_: str, request_headers: Headers) -> None:
    """
    Enable CORS.
    """
    request_headers['Access-Control-Allow-Origin'] = '*'
