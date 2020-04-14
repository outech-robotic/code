"""
Dependency injector module.
"""
import inspect
from typing import Any, Dict, Callable

from src.logger import LOGGER


class DependencyContainer:
    """
    DependencyContainer is a class that resolves all the dependencies of the project (it 
    basically does dependency injection).
    
    Example: If you have a class `Foo`, that needs a class `Bar`:
    > class Bar:
    >     [...]
    >
    > class Foo:
    >     def __init__(self, bar):
    >         self.bar = bar
    >     [...]
        
    You can inject it using the DependencyContainer:
    > i = DependencyContainer()
    > i.provide('bar', Bar) # 'bar' is the argument name in Foo's constructor.
    > i.provide('foo', Foo)
    >
    > foo = i.get('foo') # Foo's constructor is called with an instance from `Bar`.

    Bar can also have dependencies in its constructor that will be resolved by the injector.
    """

    def __init__(self) -> None:
        self.instances: Dict[str, Any] = {}
        self.factories: Dict[str, Callable] = {}
        self.args: Dict[str, Any] = {}

    def provide(self, name: str, cls: Any, **args: Any) -> None:
        """
        Provide takes a name and a class. An instance will be created for `class`. This instance 
        will be provided to the classes that have an argument `name` in their constructor.
        
        You can also pass additional arguments that will be passed when instantiating the `class`.
        
        Instead of a class, you can also pass an instance of a class.
        """
        if name in self.factories:
            raise RuntimeError(f"{name} already provided")

        if callable(cls):
            if inspect.isabstract(cls):
                raise RuntimeError(
                    f"{cls} ({name}) is abstract and cannot be instantiated")
            self.factories[name] = cls
            self.args[name] = args
        else:
            self.instances[name] = cls

    def get(self, name: str) -> Any:
        """
        Get an instance of a class with its dependencies resolved.
        """
        if name in self.instances:
            return self.instances[name]

        if name not in self.factories:
            raise RuntimeError(
                f"'{name}' never provided, add i.provide('{name}') in main.py")

        cls = self.factories[name]

        args_names = list(inspect.signature(cls).parameters.keys())

        args = {}
        for arg_name in args_names:
            if arg_name in self.args[name]:
                value = self.args[name][arg_name]
            else:
                value = self.get(arg_name)

            if value is None:
                raise RuntimeError(f"{arg_name} not provided, needed by {name}")
            args[arg_name] = value

        LOGGER.get().info('injecting_dependency', name=name)
        self.instances[name] = cls(**args)
        return self.instances[name]
