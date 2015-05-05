#!/usr/bin/env python
"""
Test cases for Quaternion Object
"""
import unittest
import numpy
import numpy.testing
from ardrone_lib import quadrotor

class TestQuadrotor(unittest.TestCase):
    """Quadrotor module test cases"""

    def setUp(self):
        self._name = 'test'
        self._quadrotor = quadrotor.Quadrotor(self._name)
    def test_one_equals_one(self):
        """Test 1=1"""
        self.assertEqual(1, 1, "1!=1")
    def test_get_name(self):
        """test get_name method"""
        self.assertEqual(self._quadrotor.get_name(), self._name)
    def test_setget_position(self):
        """test set_position and get_position methods"""
        self._quadrotor.set_position(1.0, -1.0, 0.25)
        numpy.testing.assert_array_equal(
            self._quadrotor.get_position(), numpy.array([1.0, -1.0, 0.25]))
    def test_setget_velocity(self):
        """test set_velocity, set_omega and get_velocity methods"""
        self._quadrotor.set_velocity(1.0, -1.0, 0.25)
        numpy.testing.assert_array_equal(
            self._quadrotor.get_velocity(), numpy.array([1.0, -1.0, 0.25, 0., 0., 0.]))
        self._quadrotor.set_omega(0.25, -1.5, 0.33)
        numpy.testing.assert_array_equal(
            self._quadrotor.get_velocity(), numpy.array([1.0, -1.0, 0.25, 0.25, -1.5, 0.33]))
    def test_setget_rotation(self):
        """test set_rotation and get_rotation method"""
        self._quadrotor.set_rotation(0.0, 5*numpy.pi/2, -numpy.pi/2)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.get_rotation(), numpy.array([0.0, numpy.pi/2, -numpy.pi/2]))
    def test_setget_altitude(self):
        """test set_altitude and get_altitude"""
        self._quadrotor.set_altitude(1.0)
        self.assertEqual(self._quadrotor.get_altitude(), 1.0)
    def test_setget_heading(self):
        """test set_heading and get_heading"""
        self._quadrotor.set_heading(numpy.pi)
        self.assertEqual(self._quadrotor.get_heading(), numpy.pi)
        self._quadrotor.set_heading(3*numpy.pi)
        self.assertAlmostEqual(self._quadrotor.get_heading(), numpy.pi)
    def test_setget_battery(self):
        """test set_battery and get_battery method"""
        for battery_percentage in range(1, 101):
            self._quadrotor.set_battery(battery_percentage)
            self.assertEqual(self._quadrotor.get_battery(), battery_percentage)
            self._quadrotor.set_battery(battery_percentage-0.5)
            self.assertEqual(self._quadrotor.get_battery(), battery_percentage-0.5)
    def test_battery_range(self):
        """test raise errors"""
        self.assertRaises(ValueError, self._quadrotor.set_battery, 101.)
        self.assertRaises(ValueError, self._quadrotor.set_battery, -1.)
    def test_setget_status(self):
        """test set_status and get_status method"""
        for status in range(0, len(quadrotor.STATUS)):
            self._quadrotor.set_status(status)
            self.assertEqual(self._quadrotor.get_status(), status)
    def test_status_range(self):
        """test raise errors"""
        self.assertRaises(IndexError, self._quadrotor.set_status,
                          len(quadrotor.STATUS))
        self.assertRaises(IndexError, self._quadrotor.set_status, -1.)
    def test_local_to_global(self):
        """test local to global rotation"""
        x = numpy.array([1., 0., 0.])
        y = numpy.array([0., 1., 0.])
        z = numpy.array([0., 0., 1.])

        self._quadrotor.set_heading(numpy.pi/2)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.local_to_global_rotation(x), y)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.local_to_global_rotation(y), -x)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.local_to_global_rotation(z), z)

        self._quadrotor.set_heading(numpy.pi)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.local_to_global_rotation(x), -x)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.local_to_global_rotation(y), -y)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.local_to_global_rotation(z), z)

        self._quadrotor.set_heading(-numpy.pi/2)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.local_to_global_rotation(x), -y)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.local_to_global_rotation(y), x)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.local_to_global_rotation(z), z)
    def test_global_to_local(self):
        """test global to local rotation"""
        x = numpy.array([1., 0., 0.])
        y = numpy.array([0., 1., 0.])
        z = numpy.array([0., 0., 1.])

        self._quadrotor.set_heading(numpy.pi/2)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.global_to_local_rotation(x), -y)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.global_to_local_rotation(y), x)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.global_to_local_rotation(z), z)

        self._quadrotor.set_heading(numpy.pi)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.global_to_local_rotation(x), -x)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.global_to_local_rotation(y), -y)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.global_to_local_rotation(z), z)

        self._quadrotor.set_heading(-numpy.pi/2)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.global_to_local_rotation(x), y)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.global_to_local_rotation(y), -x)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.global_to_local_rotation(z), z)
    def test_pos_odometric_prediction(self):
        """test the odometric prediction of position"""
        self._quadrotor.set_heading(numpy.pi/4)
        self._quadrotor.set_velocity(1.0, -1.0, 0.5)
        self._quadrotor.odometric_prediction(1.0)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.get_position(), numpy.array([2**0.5, 0.0, 0.5]))
        self._quadrotor.set_velocity(-1.0, +1.0, -0.5)
        self._quadrotor.odometric_prediction(1.0)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.get_position(), numpy.array([0., 0., 0.]))
        self._quadrotor.set_heading(numpy.pi)
        self._quadrotor.set_velocity(0.5, -1.0, -0.5)
        self._quadrotor.odometric_prediction(1.0)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.get_position(), numpy.array([-0.5, 1.0, -0.5]))
        self._quadrotor.set_velocity(-0.5, 1.0, 0.5)
        self._quadrotor.odometric_prediction(1.0)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.get_position(), numpy.array([0., 0., 0.]))
        self._quadrotor.set_heading(-numpy.pi/2)
        self._quadrotor.set_velocity(0.5, 1.0, 0.0)
        self._quadrotor.odometric_prediction(1.0)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.get_position(), numpy.array([1.0, -0.5, 0.0]))
    def test_rot_odometric_prediction(self):
        """test the odometric prediction of rotation"""
        self._quadrotor.set_heading(0.)
        self._quadrotor.set_omega(0., 0., 1.)
        self._quadrotor.odometric_prediction(1.0)
        numpy.testing.assert_array_almost_equal(
            self._quadrotor.get_rotation(), numpy.array([0.0, 0., 1.0]))
    def test_odometric_prediction_range(self):
        """test the odometric prediction delta_t parameter"""
        self.assertRaises(ValueError, self._quadrotor.odometric_prediction, -0.5)


if __name__ == '__main__':
    import rostest
    PKG = 'test_ardrone_lib'
    rostest.rosrun(PKG, 'test_quadrotor', TestQuadrotor)
