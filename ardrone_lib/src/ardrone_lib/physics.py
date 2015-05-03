#!/usr/bin/env python
"""
Physics Implemenation for Simulator
Communication delays and Systems dynamics are considered
"""
from collections import deque
from scipy.signal import ss2tf, tf2ss, cont2discrete

class Filter(object):
    """docstring for Delay"""
    def __init__(self, num, den, time_delays):
        super(Filter, self).__init__()
        self._num = num
        self._den = den
        self._input = deque([0]*len(num), maxlen=len(num))
        self._output = deque([0]*len(den), maxlen=len(den))
        self._delays = deque([0]*int(time_delays+1), maxlen=int(time_delays+1))

    def __str__(self):
        return 'num:' + str(self._num) \
             + ' den:' + str(self._den) \
            + ' delays:' + str(len(self._delays))

    def set_input(self, new_input):
        """
        Set Controller input
        """
        self._delays.append(new_input)
        self._input.append(self._delays[0])

    def calculate_output(self):
        """
        Calculate Controller output
        """
        new_output = 0
        for idx in range(len(self._num)):
            new_output += self._num[idx] * self._input[-1 - idx]
        for idx in range(len(self._den)):
            new_output -= self._den[idx] * self._input[-1 - idx]
        self._output.append(new_output)

    def get_output(self):
        """
        Get system current output
        """
        return self._output[-1]

class QuadrotorPhysics(object):
    """docstring for Quadrotor"""
    def __init__(self, sim_time, delay):
        super(QuadrotorPhysics, self).__init__()
        continuous_systems = {
            'x': tf2ss([6.9], [1., 0.2]),
            'y': tf2ss([4.774], [1., 0.4596]),
            'z': tf2ss([0.1526, 5.153], [1.0, 5.82]),
            'yaw': tf2ss([2.4], [1, 2.0])
        }
        self._system = {}
        for key, lti_sys in continuous_systems.items():
            A, B, C, D, dt = cont2discrete(lti_sys, sim_time)
            num, den = ss2tf(A, B, C, D)
            self._system[key] = Filter(
                num.reshape(-1,).tolist(),
                den.reshape(-1,).tolist(),
                delay/sim_time)

        # self._maximum = {
        #     'x': rospy.get_param('euler_angle_max') ,
        #     'y': rospy.get_param('euler_angle_max'),
        #     'z': rospy.get_param('control_vz_max'),
        #     'yaw': rospy.get_param('control_yaw')
        # }


    def set_input(self, input_dict):
        """
        Set new input
        """
        for axis, system in self._system.items():
            system.set_input(input_dict[axis])# * self._maximum[axis])

    def get_output(self):
        """
        Get System Output
        """
        return {key: system.get_output() for key, system in self._system.items()}

