#!/usr/bin/env python

"""
Quaternion Module to handle orientation
"""
from numpy import array, allclose, random, sin, cos, sqrt, pi, exp, append
from numpy.linalg import norm
import tf

class Quaternion(object):
    """
    Object that has x,y,z,w properties for each quaternion.
    """
    def __init__(self, q_x=0., q_y=0., q_z=0., q_w=1.):
        super(Quaternion, self).__init__()
        self._quaternion = array([q_x, q_y, q_z, q_w])
        #self.normalize()
    def __str__(self):
        return str(self._quaternion)

    def __iter__(self):
        for direction in self._quaternion:
            yield direction

    def __add__(self, other):
        qx1, qy1, qz1, qw1 = self
        qx2, qy2, qz2, qw2 = other
        return Quaternion(qx1+qx2, qy1+qy2, qz1+qz2, qw1+qw2)

    def __sub__(self, other):
        qx1, qy1, qz1, qw1 = self
        qx2, qy2, qz2, qw2 = other
        return Quaternion(qx1-qx2, qy1-qy2, qz1-qz2, qw1-qw2)

    def __mul__(self, other):
        if type(self) is type(other):
            q_x, q_y, q_z, q_w = tf.transformations.quaternion_multiply(
                self.get_quaternion(), other.get_quaternion())
            return Quaternion(q_x, q_y, q_z, q_w)
        else:
            q_x, q_y, q_z, q_w = self
            return Quaternion(other*q_x, other*q_y, other*q_z, other*q_w)

    def __eq__(self, other):
        return allclose(self.get_quaternion(), other.get_quaternion())

    def exponential(self):
        """Return the exponential of the quaternion"""
        other = self.clone()
        other.exponentiate()
        return other

    def exponentiate(self):
        """Exponentiate the quaternion"""
        vector = self.get_vector()
        vector_norm = norm(vector)
        scalar = self.get_scalar()
        if vector_norm != 0:
            new_vector = sin(vector_norm)/vector_norm * vector
        else:
            new_vector = vector
        new_scalar = cos(vector_norm)
        self._quaternion = exp(scalar)*append(new_vector, new_scalar)

    def conjugated(self):
        """
        Return conjugate Quaternion
        """
        new_quaternion = self.clone()
        new_quaternion.conjugate()
        return new_quaternion

    def conjugate(self):
        """
        Conjugate Quaternion
        """
        self._quaternion = tf.transformations.quaternion_conjugate(self._quaternion)

    def inverse(self):
        """
        Return inversed Quaternion
        """
        new_quaternion = self.clone()
        new_quaternion.invert()
        return new_quaternion

    def is_null(self):
        """Check if quaternion is null"""
        ans = True
        for number in self:
            ans &= number == 0
        return ans

    def invert(self):
        """
        Invert Quaternion
        """
        self._quaternion = tf.transformations.quaternion_inverse(self._quaternion)

    def normalize(self):
        """
        Normalize the quaternion to yield a unit quaternion
        """
        if not self.is_null():
            self._quaternion = self._quaternion/norm(self._quaternion)

    def jacobian(self, axis):
        """
        Return jacobian matrix for positive rotation along axis
        """
        q_x, q_y, q_z, q_w = self
        if axis == 'x':
            return 2 * array([[0., -2*q_y, -2*q_z, 0.],
                              [q_y, q_x, q_w, q_z],
                              [q_z, -q_w, q_x, -q_y]])
        elif axis == 'y':
            return 2 * array([[q_y, q_x, -q_w, -q_z],
                              [-2*q_x, 0., -2*q_z, 0.],
                              [q_w, q_z, q_y, q_x]])
        elif axis == 'z':
            return 2 * array([[q_z, q_w, q_x, q_y],
                              [-q_w, q_z, q_y, -q_x],
                              [-2*q_x, -2*q_y, 0., 0.]])

    def inv_jacobian(self, axis):
        """
        Return jacobian matrix for inverse rotation along axis
        """
        new_quaternion = self.clone()
        new_quaternion.invert()
        return - new_quaternion.jacobian(axis)

    def set_quaternion(self, q_x, q_y, q_z, q_w):
        """
        Set Quaternion
        """
        self._quaternion = array([q_x, q_y, q_z, q_w])

    def get_quaternion(self):
        """
        Get Quaternion
        """
        return self._quaternion

    def set_euler(self, roll, pitch, yaw):
        """
        Set roll, pitch, yaw angles
        """
        self._quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    def get_euler(self):
        """
        Get euler angles
        """
        return tf.transformations.euler_from_quaternion(self._quaternion)

    def get_rotation_matrix(self):
        """
        Get rotation matrix of configuration
        """
        return tf.transformations.quaternion_matrix(self.get_quaternion())[0:3, 0:3]

    def update(self, omega_x, omega_y, omega_z, delta_t):
        """
        Update the current state of the quaternion
        Given angular velocities
        """
        omega = Quaternion(omega_x, omega_y, omega_z, 0.) * (delta_t*0.5)
        omega.exponentiate()
        omega.normalize()
        self._quaternion = (omega * self).get_quaternion()
        #self._quaternion = ((qdot * delta_t)+self).get_quaternion()
        #self.normalize()

    def get_vector(self):
        """Return vector part of quaternion"""
        return self._quaternion[0:3]

    def get_scalar(self):
        """Return scalar part of quaternion"""
        return self._quaternion[3]

    def clone(self):
        """
        Return a copy of itself
        """
        q_x, q_y, q_z, q_w = self
        return Quaternion(q_x, q_y, q_z, q_w)

def random_quaternion():
    """Return uniform random unit quaternion."""
    rand = random.rand(3)
    r_1 = sqrt(1.0 - rand[0])
    r_2 = sqrt(rand[0])
    t_1 = 2.0 * pi * rand[1]
    t_2 = 2.0 * pi * rand[2]
    return Quaternion(sin(t_1)*r_1,
                      cos(t_1)*r_1,
                      sin(t_2)*r_2,
                      cos(t_2)*r_2)

def test():
    """
    testing
    """
    print random_quaternion()
    q_1 = Quaternion(1., -2., 3., 4.)
    q_2 = Quaternion(-5., 6., 7., 8.)
    print q_1*q_2
    print q_1*q_2 == Quaternion(-44., -14., 48., 28.)
    print q_1 + q_2 == Quaternion(-4., 4., 10., 12.)
    print q_1
    print q_1.conjugated()
    print q_1
    print q_1.get_euler()
    q_1.normalize()
    print q_1 * 30**0.5

if __name__ == '__main__':
    test()

