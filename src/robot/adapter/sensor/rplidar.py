from src.robot.controller.sensor.rplidar import LidarController
from src.simulation.controller.probe import SimulationProbe
import serial.tools.list_ports as port_list
from math import pi
import threading
import rplidar


class RplidarAdapter:

    def __init__(self, lidar_controller: LidarController, simulation_probe: SimulationProbe):
        self.lidar_controller = lidar_controller
        self.simulation_probe = simulation_probe
        self.lidar = None
        simulation_probe.attach("lidar", lambda: self.lidar is not None)
        self.__init_lidar()
        self.__start_thread_lidar_controller()

    def __init_lidar(self):
        if self.lidar is None:
            try:
                self.lidar = rplidar.RPLidar(port_list.comports()[0].device)
                self.lidar.stop()  # stop lidar if running
                self.lidar.stop_motor()
            except Exception:
                self.lidar = None

    def __loop(self):
        try:
            if self.lidar is not None:
                for scans in self.lidar.iter_scans(scan_type='express', max_buf_meas=3500):  # boucle infinie
                    self.lidar_controller.reset_detection()
                    for _, a, r in scans:
                        a = a*pi/180  # to radian
                        self.lidar_controller.append_detection((a, r))
            else:
                while True:
                    self.lidar_controller.set_detection([(500, 500), (1000, 1000)])
        except Exception:
            self.__stop_lidar()

    def __start_thread_lidar_controller(self):
        thread_lidar = threading.Thread(target=self.__loop)
        thread_lidar.start()

    def __stop_lidar(self):
        if self.lidar is not None:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
