#!/usr/bin/env python
"""
Test cases for Sensors Objects
"""
PKG = 'test_ardrone_lib'

import unittest
from ardrone_lib import sensors
import numpy

SENSOR_DATA = {
    'bias': 0.5,
    'noise': 0.1,
    'cross_coupling': 0.1,
    'min': float('-inf'),
    'max': float('+inf')
    }

class TestSensors(unittest.TestCase):
    """docstring for ClassName"""
    def setUp(self):
        self._sensors = {
            'sen': sensors.Sensor(2),
            'gyr': sensors.Gyroscope(),
            'vel': sensors.Velocity(),
            'acc': sensors.Accelerometer(),
            'mag': sensors.Magnetometer(SENSOR_DATA),
            'gps': sensors.GPS(),
            'alt': sensors.Altimeter(),
            'marg': sensors.Marg()
        }
        self._dims = {
            'sen': 2,
            'gyr': 3,
            'vel': 2,
            'acc': 3,
            'mag': 3,
            'gps': 3,
            'alt': 1,
            'marg': 4
        }

    def test_one_equals_one(self):
        """test 1=1"""
        self.assertEquals(1, 1, "1!=1")
    def test_setget_true(self):
        """test set, get true value"""
        for name, sensor in self._sensors.items():
            true_val = numpy.random.normal(0, 1., (1, self._dims[name]))[0]
            sensor.set_true_value(true_val)
            #numpy.testing.assert_array_almost_equal(true_val, sensor.get_output())
            print sensor.get_output(), sensor.get_simulation()

if __name__ == '__main__':
    #import rostest
    #rostest.rosrun(PKG, 'test_sensors', TestSensors)
    unittest.main()
