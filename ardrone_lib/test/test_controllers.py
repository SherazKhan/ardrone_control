#!/usr/bin/env python
"""
Test cases for Implemented Controllers
"""
PKG = 'test_ardrone_lib'

import unittest
from ardrone_lib import controllers

class TestSanity(unittest.TestCase):
    """Test Implemented Controllers"""
    def setUp(self):
        """Init All Controllers"""
        self._controllers = {
            'pid': controllers.PID(1.0, 0.5, 2.0),
            'trajectory': controllers.TrajectoryPID(1.0, 0.5, 2.0),
            'digital': controllers.Digital([1.0], [1.0, 1.0], dt=0.01)
        }
        self._output = {'pid': 3.3, 'trajectory': 1.5, 'digital': 0.00995016625083}

    def test_one_equals_one(self):
        """Test 1=1"""
        self.assertEquals(1, 1, "1!=1")

    def test_sanity(self):
        """sanity checks in implemented controllers"""
        for controller in self._controllers.values():
            controller.set_saturated(True)
            self.assertTrue(controller.is_saturated())
            controller.set_saturated(False)
            self.assertFalse(controller.is_saturated())

            controller.set_periodic(True)
            self.assertTrue(controller.is_periodic())
            controller.set_periodic(False)
            self.assertFalse(controller.is_periodic())

    def test_types(self):
        """test types in input functions"""
        for controller in self._controllers.values():
            for other_type in [int, float]:
                self.assertRaises(TypeError, controller.set_saturated, other_type(True))
                self.assertRaises(TypeError, controller.set_periodic, other_type(False))

    def test_get_output(self):
        """test get output"""
        for name, controller in self._controllers.items():
            controller.set_saturated(False)
            controller.set_reference(1.0)
            self.assertAlmostEqual(controller.get_output(), self._output[name])

if __name__ == '__main__':
    #import rostest
    #rostest.rosrun(PKG, 'test_controllers', TestSanity)
    unittest.main()
