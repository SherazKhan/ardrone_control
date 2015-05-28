#!/usr/bin/env python
"""
Trajectory commander for ardrone
"""
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy #dont forget to rosrun joy joy_node !

from ardrone_lib import quadrotor
from ardrone_lib import joysticks

import scipy.signal

import numpy

COMMAND = dict(
    cross='takeoff',
    triangle='land',
    square='stop',
    R1='chirp_x',
    R2='chirp_y',
    L1='chirp_z',
    L2='chirp_yaw'
    )

AXIS = ['x', 'y', 'z', 'yaw']
X = 0
Y = 1
Z = 2
YAW = 3

class SignalResponse(object):
    """docstring for Signal"""
    def __init__(self, delta_t):
        super(SignalResponse, self).__init__()
        self._output = dict()
        self._axis = dict()
        self._amplitude = rospy.get_param('amplitude', 0.1)
        for axis in AXIS:
            self._output[axis] = numpy.array([])
            self._axis[axis] = False

        self._dt = delta_t

    def set_axis(self, axis):
        """ set chirp axis """
        if axis not in self._axis.keys():
            raise KeyError
        else:
            self._axis[axis] = True
            self._generate_chirp(axis,
                                 rospy.get_param('chirp_time'),
                                 rospy.get_param('f_initial'),
                                 rospy.get_param('f_final'))

    def unset_axis(self, axis):
        """ unset chirp axis """
        if axis not in self._axis.keys():
            raise KeyError
        else:
            self._axis[axis] = False

    def get_output(self):
        """ iterate over output """
        for key, output in self._output.items():
            if self._axis[key]:
                if len(output) > 0:
                    new_output = output[0]
                    self._output[key] = numpy.delete(output, 0)
                    yield key, new_output
                else:
                    self.unset_axis(key)

    def _generate_chirp(self, axis, chirp_time, f_initial, f_final):
        """generate chirp signal """
        time_array = numpy.linspace(0., chirp_time, num=numpy.floor(chirp_time/self._dt + 1))
        self._output[axis] = scipy.signal.chirp(time_array, f_initial, chirp_time, f_final)
        self._output[axis] *= self._amplitude

class TrajectoryCommander(object):
    """
    The trajectory commander handles the drone state.
    It sends take-off, land, emergency and stop messages
    It sets close loop or open loop commanding
    It updates the reference for the closed loop controller.
    """
    def __init__(self, joystick):
        super(TrajectoryCommander, self).__init__()
        self._joystick = joystick
        self._sequencer = 0
        delta_t = rospy.get_param('processing_time')
        self._signal = SignalResponse(delta_t)
        self._publishers = {
            'land': rospy.Publisher('ardrone/land', Empty, queue_size=1),
            'take_off': rospy.Publisher('ardrone/takeoff', Empty, queue_size=1),
            'reset': rospy.Publisher('ardrone/reset', Empty, queue_size=1),
            'cmd': rospy.Publisher('cmd_vel', Twist, queue_size=10./delta_t)
            }

        rospy.Subscriber('/joy', Joy, callback=self.recieve_joystick)

        rospy.Timer(rospy.Duration(delta_t), self.command_chirp)

    def land(self, dummy_arg=None):
        """Land Drone"""
        self._publishers['land'].publish()

    def takeoff(self, dummy_arg=None):
        """Land Drone"""
        self._publishers['take_off'].publish()

    def stop(self):
        """Send Drone to Hover Mode"""
        self.command_vel(Twist())

    def chirp_x(self):
        """start chirp x callback"""
        self._chirp(AXIS[X])
    def chirp_y(self):
        """start chirp x callback"""
        self._chirp(AXIS[Y])
    def chirp_z(self):
        """start chirp x callback"""
        self._chirp(AXIS[Z])
    def chirp_yaw(self):
        """start chirp x callback"""
        self._chirp(AXIS[YAW])

    def _chirp(self, axis):
        """start chirp wrapper"""
        self._signal.set_axis(axis)
        rospy.loginfo('Chirp in %s axis', axis)

    def command_chirp(self, dummy_time_event):
        """Set the reference to the ardrone"""
        msg = Twist()
        for axis, outval in self._signal.get_output():
            if axis == AXIS[X]:
                msg.linear.x = outval
            elif axis == AXIS[Y]:
                msg.linear.y = outval
            elif axis == AXIS[Z]:
                msg.linear.z = outval
            elif axis == AXIS[YAW]:
                msg.angular.z = outval
        self.command_vel(msg)

    def command_vel(self, twist_msg):
        """Send Local velocity to Drone"""
        self._publishers['cmd'].publish(twist_msg)

    def recieve_joystick(self, joy_msg):
        """Parse joystick message and run appropiate method"""
        idx = 0
        for is_pressed in getattr(joy_msg, 'buttons'):
            if is_pressed:
                button = self._joystick['buttons'][idx]
                try:
                    getattr(self, COMMAND[button])()
                except KeyError:
                    pass
                except IndexError:
                    pass
            idx += 1


if __name__ == '__main__':
    rospy.init_node('trayectory', anonymous=True)
    TrajectoryCommander(joysticks.PS3)
    rospy.spin()
