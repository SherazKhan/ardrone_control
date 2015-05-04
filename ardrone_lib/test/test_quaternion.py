#!/usr/bin/env python
"""
Test cases for Quaternion Object
"""
import unittest
from ardrone_lib import quaternion
import numpy.testing
#from numpy import array, exp

class TestQuaternion(unittest.TestCase):
    """docstring for ClassName"""
    def setUp(self):
        self._random = quaternion.random_quaternion()
        self._q1 = quaternion.Quaternion(1., -2., 3., 4.)
        self._q2 = quaternion.Quaternion(-5., 6., 7., 8.)
        self._null = quaternion.Quaternion(0., 0., 0., 0.)
        self._unit = quaternion.Quaternion(0., 0., 0., 1.)
    def test_one_equals_one(self):
        """Test 1=1"""
        self.assertEquals(1, 1, "1!=1")
    def test_add(self):
        """add 2 quaternions"""
        self.assertEqual(self._q1 + self._q2,
                         quaternion.Quaternion(-4., 4., 10., 12.))
    def test_sub(self):
        """substract 2 quaternions"""
        self.assertEqual(self._q1 - self._q2,
                         quaternion.Quaternion(6, -8, -4, -4))
        self.assertEqual(self._q2 - self._q1,
                         quaternion.Quaternion(-6, 8, 4, 4))
        self.assertEqual(self._q1 - self._q1, self._null)
        self.assertEqual(self._q2 - self._q2, self._null)
    def test_mul(self):
        """multiply 2 quaternions and a float by a quaternions"""
        self.assertEqual(self._q1 * self._q2, quaternion.Quaternion(-44., -14., 48., 28.))
        self.assertEqual(self._q1 * 2, quaternion.Quaternion(2., -4., 6., 8.))
        self.assertEqual(self._q1 * (-0.5), quaternion.Quaternion(-0.5, 1., -1.5, -2.))
    def test_is_null(self):
        """test if quaternion is null"""
        self.assertTrue(self._null.is_null())
        self.assertFalse(self._unit.is_null())
        self.assertFalse(self._random.is_null())
    def test_conjugated(self):
        """test conjugated quaternion"""
        self.assertTrue((
            self._random.conjugated().get_quaternion()[0:3] == -self._random.get_quaternion()[0:3]
            ).all())
        self.assertTrue(
            self._random.conjugated().get_quaternion()[3] == self._random.get_quaternion()[3])
    def test_conjugate(self):
        """conjugate the quaternion"""
        other = self._random.conjugated()
        self._random.conjugate()
        self.assertEqual(self._random, other)
    def test_exponential(self):
        """test exponential of quaternion"""
        self.assertTrue(self._unit.exponential() == quaternion.Quaternion(0., 0., 0., numpy.exp(1.)))
    def test_exponentiate(self):
        """exponentiate the quaternion"""
        self._unit.exponentiate()
        self.assertTrue(self._unit == quaternion.Quaternion(0., 0., 0., numpy.exp(1.)))
    def test_inverse(self):
        """test inverse quaternion"""
        self.assertEqual(self._random.inverse() * self._random, self._unit)
    def test_invert(self):
        """invert the quaternion"""
        other = self._random.inverse()
        self._random.invert()
        self.assertEqual(other, self._random)
    def test_jacobian(self):
        """get the jacobian"""
        pass
    def test_inverse_jacobian(self):
        """get the jacobian of the inverse rotation"""
        pass
    def test_setget_quaternion(self):
        """test set_quaternion, get_quaternion methods"""
        self._random.set_quaternion(0.5, -2.0, 3.0, -0.93)
        numpy.testing.assert_array_almost_equal(
            self._random.get_quaternion(), numpy.array([0.5, -2.0, 3.0, -0.93]))
    def test_get_vector(self):
        """test get_vector"""
        numpy.testing.assert_array_almost_equal(
            self._random.get_quaternion()[0:3], self._random.get_vector())
    def test_get_scalar(self):
        """test get_scalar"""
        numpy.testing.assert_array_almost_equal(
            self._random.get_quaternion()[3], self._random.get_scalar())
    def test_setget_euler(self):
        """test set_euler, get_euler methods"""
        self._random.set_euler(1.0, -1.0, 0.5)
        numpy.testing.assert_array_almost_equal(
            self._random.get_euler(), numpy.array([1.0, -1.0, 0.5]))
    def test_update(self):
        """test the update of the quaternion"""
        self._unit.update(0, 0, 1, 1)
        self.assertEqual(self._unit.get_euler(), (0, 0, 1.))
        self._unit.update(0, 0, -0.5, 2)
        self.assertEqual(self._unit.get_euler(), (0, 0, 0.))

        self._unit.update(0, 1, 0, 1)
        self.assertEqual(self._unit.get_euler(), (0, 1, 0.))
        self._unit.update(0, -2., 0, 0.5)
        self.assertEqual(self._unit.get_euler(), (0, 0, 0.))

        self._unit.update(1, 0, 0, 1)
        self.assertEqual(self._unit.get_euler(), (1, 0, 0.))
        self._unit.update(-2, 0., 0, 0.5)
        self.assertAlmostEqual(self._unit.get_euler(), (0, 0, 0.))
    def test_clone(self):
        """test the clone method"""
        other = self._random.clone()
        self.assertFalse(other is self._random)

if __name__ == '__main__':
    unittest.main()
