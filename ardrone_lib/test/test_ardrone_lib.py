#!/usr/bin/env python
"""
Test all modules
"""
PKG = 'test_ardrone_lib'
from test_controllers import TestControllers
from test_quadrotor import TestQuadrotor
from test_quaternion import TestQuaternion
from test_filters import TestFilters
import rostest

if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_controllers', TestControllers)
    rostest.rosrun(PKG, 'test_quadrotor', TestQuadrotor)
    rostest.rosrun(PKG, 'test_quaternion', TestQuaternion)
    rostest.rosrun(PKG, 'test_filters', TestFilters)
