#!/usr/bin/env python
"""
Test cases for Quaternion Object
"""
PKG = 'test_ardrone_lib'

import unittest

class TestQuaternion(unittest.TestCase):
    """docstring for ClassName"""

    def test_one_equals_one(self):
        """
        Test 1=1
        """
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_quaternion', TestQuaternion)
