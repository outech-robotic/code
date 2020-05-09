"""Code OUTech
Highlevel
"""
import argparse
import os
from os import _Environ
from typing import Dict, Any


def _get_arguments() -> Dict[str, Any]:
    """
    Set default values for script arguments.
    """
    argument_parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    return vars(argument_parser.parse_args())


def _get_env_variables() -> _Environ:
    """
    Set environment variables.
    """
    env_variables = os.environ
    return env_variables


class CommandLineParser:
    """
    Parse arguments and environment variables from command line.
    """
    def __init__(self) -> None:
        self.args = _get_arguments()
        self.env_variables = _get_env_variables()

    def get_arguments(self) -> Dict[str, Any]:
        """
        @use: file = get_arguments()['file']
        """
        return self.args

    def get_env_var(self, variable: str) -> str:
        """
        @use: if get_local('OUTECH_SIMULATION') == 'true'
        """
        return self.env_variables.get(variable, '').lower()

    def get_bool_env_var(self, variable: str) -> bool:
        """
        @use: if get_local('OUTECH_SIMULATION')
        """
        return self.env_variables.get(variable, '').lower() == 'true'
