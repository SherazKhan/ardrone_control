#!/usr/bin/env python
"""
Trajectory commander for ardrone
"""
import rospy
from ardrone_msgs.msg import QuadrotorState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty
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

CHIRP_TIME = 10


class SignalResponse(object):
    """docstring for Signal"""
    def __init__(self, delta_t):
        super(Signal, self).__init__()
        self._output =  {'x': [], 'y': [], 'z': [], 'yaw': []}
        self._axis = {'x': False, 'y': False, 'z': False, 'yaw': False}
        self._dt = delta_t

    def set_axis(self, axis, chirp_time=CHIRP_TIME, f0=0.01, f1=1.0):
        if axis not in self._axis.keys():
            raise KeyError
            return
        self._axis[axis] = True
        self.generate_chirp(axis, chirp_time, f0, f1)

    def unset_axis(self, axis):
        if axis not in self._axis.keys():
            raise KeyError
            return
        self._axis[axis] = False

    def get_output(self):
        for key, output in self._output.items():
            if self._axis[key] and len(output) > 0:
                yield output.pop(0)   

    def generate_chirp(axis, chirp_time, f0, f1):
        time_array = numpy.linspace(0., chirp_time, num=numpy.floor(chirp_time/self._dt + 1))
        self._output[axis] = scipy.signal.chirp(time_array, f0, chirp_time, f1)

class TrajectoryCommander(object):
    """
    The trajectory commander handles the drone state.
    It sends take-off, land, emergency and stop messages
    It sets close loop or open loop commanding
    It updates the reference for the closed loop controller.
    """
    def __init__(self, name, joystick):
        super(TrajectoryCommander, self).__init__()
        self._reference = quadrotor.Quadrotor(name)
        self._joystick = joystick
        self._sequencer = 0
        delta_t = rospy.get_param('processing_time')
        self._publishers = {
            'land': rospy.Publisher('ardrone/land', Empty, queue_size=1),
            'take_off': rospy.Publisher('ardrone/takeoff', Empty, queue_size=1),
            'reset': rospy.Publisher('ardrone/reset', Empty, queue_size=1),
            'cmd': rospy.Publisher('cmd_vel', Twist, queue_size=10./delta_t)            }

        rospy.Subscriber('/joy', Joy, callback=self.recieve_joystick)

        rospy.Timer(rospy.Duration(delta_t), self.command_chirp)

    def land(self, dummy_arg=None):
        """Land Drone"""
        self._publishers['land'].publish()

    def takeoff(self, dummy_arg=None):
        """Land Drone"""
        if quadrotor.STATUS[self._reference.get_status()] == quadrotor.LANDED:
            self._publishers['take_off'].publish()

    def stop(self):
        """Send Drone to Hover Mode"""

        self.command_vel(Twist())

    def chirp_x(self):
        self._chirp('x') 
    def chirp_y(self):
        self._chirp('y') 
    def chirp_z(self):
        self._chirp('z') 
    def chirp_yaw(self):
        self._chirp('yaw') 

    def _chirp(self, axis):
        pass 
    
    def _stamp_msg(self, msg):
        """Get unstamped msg and return stamped msg"""
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = self._sequencer
        return msg

    def command_chirp(self, dummy_time_event):
        """Set the reference to the Position Feedback controller"""
        pass

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
                except KeyError, IndexError:
                    pass
            idx += 1


if __name__ == '__main__':
    rospy.init_node('trayectory', anonymous=True)
    TrajectoryCommander('ardrone', joysticks.PS3)
    rospy.spin()
