#!/usr/bin/env python
"""
Test cases for Implemented Filters
"""
PKG = 'test_ardrone_lib'

import unittest
from ardrone_lib import filters
from scipy.signal import dstep, dimpulse
class TestFilters(unittest.TestCase):
    """docstring for ClassName"""
    def test_one_equals_one(self):
        """
        Test 1=1
        """
        self.assertEquals(1, 1, "1!=1")
    def test_delay(self):
        """test a dead-time delay TF"""
        for n_delays in range(0, 10, 2):
            delay = filters.Delay(n_delays)
            delay.set_input(1.0)
            for dummy in range(n_delays):
                delay.set_input(0.0)
            self.assertEquals(delay.get_output(), 1.0)
            delay.set_input(0.0)
            self.assertEquals(delay.get_output(), 0.0)
    def test_transfer_function_step(self):
        """Test step response of transfer function"""
        delta_t = 0.01
        transfer_function = filters.TransferFunction([1.0], [1.0, 5.0], delta_t)
        time, output = dstep((transfer_function.get_num(), transfer_function.get_den(), delta_t))
        for idx in range(len(time)):
            self.assertAlmostEqual(output[0][idx], transfer_function.get_output())
            transfer_function.set_input(1.0)
    def test_transfer_function_impulse(self):
        """Test impulse response of transfer function"""
        delta_t = 0.01
        transfer_function = filters.TransferFunction([1.0], [1.0, 5.0], delta_t)
        time, output = dimpulse((transfer_function.get_num(), transfer_function.get_den(), delta_t))
        for idx in range(len(time)):
            self.assertAlmostEqual(output[0][idx], transfer_function.get_output())
            transfer_function.set_input(1.0 if idx == 0 else 0.0)
    def test_leaky_integrator_lambda0(self):
        """test leaky integrator when lambda=0.0"""
        leaky = filters.LeakyIntegrator(0.)
        for number in range(-100, 100):
            leaky.set_input(number)
            self.assertEqual(leaky.get_output(), number)
    def test_leaky_integrator_lambda1(self):
        """test leaky integrator when lambda=1.0"""
        leaky = filters.LeakyIntegrator(1.)
        for number in range(-100, 100):
            leaky.set_input(number)
            self.assertEqual(leaky.get_output(), 0.)
    def test_leaky_integrator(self):
        """test leaky integrator when lambda=0.5"""
        leaky = filters.LeakyIntegrator(0.5)
        output = 0
        for number in range(-100, 100):
            leaky.set_input(number)
            self.assertEqual(leaky.get_output(), 0.5*(number+output))
            output = leaky.get_output()
    def test_tf_with_delay(self):
        """test transfer function + delay"""
        tf_w_delay = filters.TransferFunctionWithDelay([1.0], [1.0, -0.095], 5)
        t_f = filters.TransferFunction([1.0]+[0.]*5, [1.0, -0.095])

        for dummy_idx in range(100):
            tf_w_delay.set_input(1.0)
            t_f.set_input(1.0)
            self.assertAlmostEqual(tf_w_delay.get_output(), t_f.get_output())


if __name__ == '__main__':
    #import rostest
    #rostest.rosrun(PKG, 'test_filters', TestFilters)
    unittest.main()
