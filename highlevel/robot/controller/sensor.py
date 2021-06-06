"""
Sensor Controller module
"""
import time
from typing import List

from highlevel.logger import LOGGER
from highlevel.robot.entity.configuration import Configuration
from highlevel.util.type import Millimeter

"""
y = mesure_brute_moyennee [0 ; 2¹²-1]
x = distance_associee(mm)
On suppose que la mesure est lineaire (ce qui est suffisant sur la plage choisie)
y = ax + b
->
x = (y - b) / a

Avec
a = (ymax - ymin) / (xmax - xmin) = yrange / xrange
b = ymin = valeur_brute_la_plus_courte_mesurable
ymax = valeur_brute_la_plus_longue_mesurable
xmax et xmin = distances_associees

->
x = xmin + xrange * (y-ymin)/(yrange)
"""
# Range settings in 12 bit ADC samples.
_MIN_LSB = [750, 734, 734, 734]  # Smallest sample possible.
_RANGE_LSB = [3285, 3301, 3301, 3301]

# Range settings in millimeters.
_MIN_MM = 100  # closest distance possible.
_RANGE_MM = 500
_SAMPLE_SIZE = 10

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

    def _convert_adc_sample_to_distance(self, id: int, sample: int) -> Millimeter:
        """
        Convert 12bit adc data to distances, using configuration data.
        """
        return _MIN_MM + _RANGE_MM * (sample - _MIN_LSB[id]) / _RANGE_LSB[id] \
            if sample >= _MIN_LSB[id] else 0

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
            self.laser_moving_sums = [_SAMPLE_SIZE * d for d in raw_data_list]
            self.laser_last_distances_list = []
            for i, d in enumerate(raw_data_list):
                self.laser_last_distances_list.append([d for _ in range(_SAMPLE_SIZE)])

            self.laser_distances_list = [self._convert_adc_sample_to_distance(i, s // _SAMPLE_SIZE)
                                         for i, s in enumerate(self.laser_moving_sums)]
        else:
            self._update_moving_averages(raw_data_list)
        LOGGER.get().info('sensor_controller_avg',
                          avg=[s / _SAMPLE_SIZE for s in self.laser_moving_sums],
                          data=raw_data_list)

        for i in range(len(raw_data_list)):
            self.laser_distances_list[i] = self._convert_adc_sample_to_distance(
                i,
                self.laser_moving_sums[i] // _SAMPLE_SIZE
            )
        LOGGER.get().info('sensor_controller_converted', dists=self.laser_distances_list)

        t = time.time() - t
