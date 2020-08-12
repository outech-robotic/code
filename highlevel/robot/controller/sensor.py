"""
Sensor Controller module
"""
import math
import time
from typing import Tuple, List

from highlevel.logger import LOGGER
from highlevel.robot.entity.configuration import Configuration
from highlevel.util.geometry.vector import Vector2
from highlevel.util.type import Radian, Millimeter

"""
y = mesure_brute_moyennée(0 - 65535)
x = distance_associée(mm)

y = ax + b
->
x = (y - b) / a

Avec
a = (ymax - ymin) / (xmax - xmin)
b = ymin = valeur_brute_la_plus_courte_mesurée
ymax = valeur_brute_la_plus_longue_mesurée
xmax / xmin = distances_associees

->
x = xmin + (y - ymin) / ((ymax - ymin) / (xmax - xmin))
"""
MIN = 734
MAX = 4035

class SensorController:
    """
    The sensor controller handles incoming sensor data from sensor boards.
    """

    def __init__(self, configuration: Configuration):
        self.configuration = configuration
        self.pressure_state_list: List[bool]  # true = pressure sensor activated.
        self.laser_distances_list: List[Millimeter]  # Available values to use in strategy
        # all the moving average for each raw input
        self.laser_last_distances_list: List[List[int]] = []
        self.laser_moving_sums: List[int] = []  # Current moving averages of the input data

    def update_pressure_sensors(self, state_list: List[bool]) -> None:
        """
        Updates the current pressure sensor states from incoming data.
        """
        self.pressure_state_list = state_list
        LOGGER.get().debug('sensor_controller_update_pressure',
                          data=state_list)

    def _convert_adc_sample_to_distance(self, sample: int) -> Millimeter:
        """
        Convert 12bit adc data to distances, using configuration data.
        """
        return 100+500*(sample-MIN)/(MAX-MIN) if sample >= MIN else 0

    def _update_moving_averages(self, raw_data_list: List[int]) -> None:
        """
        Shifts the data into the related lists, updates the average.
        """

        for i, d in enumerate(raw_data_list):
            self.laser_moving_sums[i] -= self.laser_last_distances_list[i][0]
            self.laser_last_distances_list[i] = self.laser_last_distances_list[i][1:]
            self.laser_last_distances_list[i].append(d)
            self.laser_moving_sums[i] += d

    def update_laser_sensors(self, raw_data_list: List[int]) -> None:
        """
        Updates the current laser sensor distances from incoming raw data.
        The data should be the raw 12 bit ADC samples, this function uses it to get Millimeters.
        """
        # Update or initialize the moving averages on the raw data
        t = time.time()

        if not self.laser_last_distances_list:
            self.laser_moving_sums = [10*d for d in raw_data_list]
            self.laser_last_distances_list = []
            for i, d in enumerate(raw_data_list):
                self.laser_last_distances_list.append([d for _ in range(10)])

            self.laser_distances_list = [self._convert_adc_sample_to_distance(s/10) for s in self.laser_moving_sums]
        else:
            self._update_moving_averages(raw_data_list)
        LOGGER.get().info('sensor_controller_avg', avg=[s/10 for s in self.laser_moving_sums], data=raw_data_list)

        for i in range(len(raw_data_list)):
            self.laser_distances_list[i] = self._convert_adc_sample_to_distance(
                self.laser_moving_sums[i]/10
            )
        LOGGER.get().info('sensor_controller_converted', dists=self.laser_distances_list)

        t = time.time() - t
        print(t, "sec")
