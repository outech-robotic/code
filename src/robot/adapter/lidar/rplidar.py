# Until we split the adapter in two, we need a way to catch exceptions for the simulation.
# pylint: disable=broad-except
"""
Lidar adapter module.
"""

import threading
from math import pi

import rplidar
import serial.tools.list_ports as port_list

from src.robot.controller.sensor.rplidar import LidarController
from src.simulation.controller.probe import SimulationProbe


class RPLIDARAdapter:
    """Rplidar adapter is an interface for receiving positions of obstacles from an rplidar."""

    def __init__(self, lidar_controller: LidarController,
                 simulation_probe: SimulationProbe):
        self.lidar_controller = lidar_controller
        self.simulation_probe = simulation_probe
        self.lidar: rplidar.RPLidar = None
        simulation_probe.attach("lidar", lambda: self.lidar is not None)
        self.__init_lidar()
        self.__start_thread_lidar_controller()

    def __init_lidar(self) -> None:
        """ Start and connect the rplidar. """
        if self.lidar is None:
            try:
                self.lidar = rplidar.RPLidar(port_list.comports()[0].device)
                self.lidar.stop()  # stop lidar if running
                self.lidar.stop_motor()
            except Exception:
                self.lidar = None

    def __loop(self) -> None:
        """ Loop to receive the obstacles from the rplidar and save them in the controller. """
        try:
            if self.lidar is not None:
                for scans in self.lidar.iter_scans(
                        scan_type='express',
                        max_buf_meas=3500):  # boucle infinie
                    self.lidar_controller.reset_detection()
                    for _, angle, radius in scans:
                        angle = angle * pi / 180  # to radian
                        self.lidar_controller.append_detection((angle, radius))
            else:
                while True:
                    self.lidar_controller.set_detection([(500, 500),
                                                         (1000, 1000)])
        except Exception:
            self.__stop_lidar()

    def __start_thread_lidar_controller(self) -> None:
        """ Start a thread to start the rplidar. """
        thread_lidar = threading.Thread(target=self.__loop)
        thread_lidar.start()

    def __stop_lidar(self) -> None:
        """ Stop the rplidar. """
        if self.lidar is not None:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
