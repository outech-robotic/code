import inspect
from dataclasses import dataclass
from typing import Dict, Callable, Optional


@dataclass
class RobotFunction:
    """
    Describe a function in one of the module of the robot.
    """
    name: str
    documentation: Optional[str]
    args: Dict[str, str]
    func: Callable


def get_methods(module):
    """
    Get the functions of a handler/controller/gateway that can be called.
    """
    functions = []
    for item in [getattr(module, x) for x in dir(module)]:
        if not callable(item) or not inspect.ismethod(item):
            continue

        function_name = item.__name__
        if function_name.startswith('_'):
            continue

        args = {}
        for name, p in inspect.signature(item).parameters.items():
            args[name] = p.annotation.__name__

        functions.append(RobotFunction(
            name=item.__qualname__,
            documentation=inspect.getdoc(item),
            args=args,
            func=item,
        ))

    return functions
