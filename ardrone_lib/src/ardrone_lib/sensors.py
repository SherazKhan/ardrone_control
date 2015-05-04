#!/usr/bin/env python
"""
Sensor Implemenation for Simulator
"""
import rospy
import utm # GPS OBJECT
import geomag #Magnetometer OBJECT
from numpy import array, ones, identity, dot
import ardrone_lib.quaternion as quaternion

import random

GRAVITY = 9.81 #X, Y, Z
H_NORTH, H_WEST, H_UP = 18.7, 2.06, 14.07
LAT, LON = -34.603530, -58.367697

class Sensor(object):
    """
    Sensor Class that considers bias, noise, cross-axis coupling and
    sensor maximum-minimum range
    The noise model used is gaussian
    """
    def __init__(self, name, dim):
        super(Sensor, self).__init__()
        data = rospy.get_param(name)
        self._bias = array([data['bias']*random.gauss(0, 0.33)]*dim)
        self._noise = array([data['noise']]*dim)
        self._true = array([0.]*dim)
        self._cross_coupling = data['cross_coupling']*(ones((dim, dim)) - identity(dim))
        self._range = (data['min'], data['max'])

    def set_true_value(self, true_val):
        """
        Set true value
        """
        self._true = true_val

    def get_output(self):
        """
        Return true value
        """
        output = self._true + random.gauss(self._bias, self._noise)
        output += dot(self._cross_coupling, output)
        for idx in range(len(output)):
            if output[idx] > self._range[1]:
                output[idx] = self._range[1]
            elif output[idx] < self._range[0]:
                output[idx] = self._range[0]
        return output

    def get_range(self):
        """
        Return sensor range
        """
        return self._range

    def set_output(self):
        """Set measured Output"""
        pass

    def get_input(self):
        """Given a measured output,
        return value that would measure_it"""
        pass

class Gyroscope(Sensor, object):
    """
    Gyroscope Sensor simulated Class
    """
    def __init__(self):
        super(Gyroscope, self).__init__('gyroscope', 3)

class Velocity(Sensor, object):
    """
    Velocity Sensor simulated Class
    Actually this is obtained by fusing accelerometer and a Lucas-Kande Tracker
    """
    def __init__(self):
        super(Velocity, self).__init__('velocity', 2)

class Accelerometer(Sensor, object):
    """
    Accelerometer Sensor simulated Class
    """
    def __init__(self):
        super(Accelerometer, self).__init__('gyroscope', 3)

    def set_orientation(self, true_quat):
        """
        Project gravity into Body Axes
        """
        acceleration = (true_quat
                        *quaternion.Quaternion(0., 0., GRAVITY, 0.)
                        *true_quat.conjugated()
                       ).get_quaternion()
        self.set_true_value(acceleration[0:3])

class Magnetometer(Sensor, object):
    """
    Magnetometer Sensor simulated Class
    """
    def __init__(self):
        super(Magnetometer, self).__init__('magnetometer', 3)

    def set_orientation(self, true_quat):
        """
        Project earth magnetic field into Body Axes
        """
        magnetometer = (true_quat.conjugated()
                        *quaternion.Quaternion(H_NORTH, H_WEST, H_UP, 0.)
                        *true_quat
                       ).get_quaternion()
        self.set_true_value(magnetometer[0:3])

class Marg(Sensor, object):
    """
    Marg Sensor simulated Class
    It does a quaternion orientation estimation
    """
    def __init__(self):
        super(Marg, self).__init__('marg', 4)

    def get_output(self):
        """
        Returned normalized output
        """
        output = super(Marg, self).get_output()
        return output/sum(output**2)

class GPS(Sensor, object):
    """
    GPS Sensor simulated Class
    """
    def __init__(self, lat=LAT, lon=LON, alt=0.):
        super(GPS, self).__init__('gps', 3)
        self._initial_utm = utm.from_latlon(lat, lon)
        self.set_true_value(array([lat, lon, alt]))

    def set_position(self, true_position):
        """
        Set True Position and update LAT, LON, ALT
        """
        lat, lon = utm.to_latlon(
            self._initial_utm[0] + true_position[0],
            self._initial_utm[1] + true_position[1],
            self._initial_utm[2],
            self._initial_utm[3])
        self.set_true_value(array([lat, lon, true_position[2]]))

class Altimeter(Sensor, object):
    """
    Sonar Altimeter Sensor simulated Class
    """
    def __init__(self):
        super(Altimeter, self).__init__('altimeter', 1)
