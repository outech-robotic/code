"""
Sensor Controller module
"""
import math
from highlevel.logger import LOGGER
from highlevel.robot.entity.configuration import Configuration
from highlevel.util.geometry.vector import Vector2
from highlevel.util.type import Radian, Millimeter
from typing import Tuple, List


class SensorController:
    """
    The sensor controller handles incoming sensor data from sensor boards.
    """

    def __init__(self, configuration: Configuration):
        self.configuration = configuration
        self.pressure_state_list: List[bool]  # true = pressure sensor activated.
        self.laser_distances_list: List[Millimeter]

    def update_pressure_sensors(self, state_list: List[bool]) -> None:
        """
        Updates the current pressure sensor states from incoming data.
        """
        self.pressure_state_list = state_list
        LOGGER.get().info('sensor_controller_update_pressure',
                          data=state_list)

    def _convert_adc_sample_to_distance(self, sample: int) -> Millimeter:
        pass

    def update_laser_sensors(self, raw_data_list: List[int]) -> None:
        """
        Updates the current laser sensor distances from incoming raw data.
        The data should be the raw 12 bit ADC samples, this function uses it to get Millimeters.
        """
        temp_distances = []
        self.laser_distances_list = temp_distances
        LOGGER.get().info('sensor_controller_update_laser',
                          data=raw_data_list)
