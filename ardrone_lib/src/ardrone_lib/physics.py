#!/usr/bin/env python
"""
Physics Implemenation for Simulator
Communication delays and Systems dynamics are considered
"""
from ardrone_lib import filters
import numpy

MAX_EULER = 0.21
MAX_Z = 0.7

class ArDrone(object):
    """docstring for ArDrone"""
    def __init__(self, sim_time, delay):
        super(ArDrone, self).__init__()
        self._systems = {
            'x': filters.TransferFunctionWithDelay(
                [6.154 * MAX_EULER], [1., 0.665], delay/sim_time, sim_time),
            'y': filters.TransferFunctionWithDelay(
                [4.774 * MAX_EULER], [1., 0.4596], delay/sim_time, sim_time),
            'z': filters.TransferFunctionWithDelay(
                [0.1526, 5.153], [1.0, 5.82], delay/sim_time, sim_time),
            'yaw': filters.TransferFunctionWithDelay(
                [1.265], [1.0], delay/sim_time, sim_time)
        }
        # self._systems = {
        #     'x': filters.TransferFunctionWithDelay(
        #         [1.0], [1.0], 0, sim_time),
        #     'y': filters.TransferFunctionWithDelay(
        #         [1.0], [1.0], 0, sim_time),
        #     'z': filters.TransferFunctionWithDelay(
        #         [1.0], [1.0], 0, sim_time),
        #     'yaw': filters.TransferFunctionWithDelay(
        #         [1.0], [1.0], 0, sim_time)
        # }

    def __iter__(self):
        for axis, system in self._systems.items():
            yield axis, system

    def __str__(self):
        string = ''
        for axis, system in self:
            string += axis + ' ' + str(system) + '\n'
        return string

    def set_input(self, twist_msg):
        """Set new input"""
        self._systems['x'].set_input(twist_msg.linear.x)
        self._systems['y'].set_input(twist_msg.linear.y)
        self._systems['z'].set_input(twist_msg.linear.z * MAX_Z)
        self._systems['yaw'].set_input(twist_msg.angular.z)

    def get_output(self):
        """Get System Output"""
        return numpy.array([
            self._systems['x'].get_output(),
            self._systems['y'].get_output(),
            self._systems['z'].get_output(),
            0.,
            0.,
            self._systems['yaw'].get_output()])

if __name__ == '__main__':
    DRONE = ArDrone(0.01, 0.2)
    for key, s in DRONE:
        print key
        output = []
        for i in range(5000):
            s.set_input(1.)
            output.append(s.get_output())
        print s.get_output()
