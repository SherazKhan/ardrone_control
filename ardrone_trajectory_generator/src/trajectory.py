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

import math


WAY_POINTS = [
    {'x': 1, 'y':0, 'z': 1, 'yaw': 0},
    {'x': 0, 'y':0, 'z': 1.5, 'yaw': 0},
    {'x': 0, 'y':1, 'z': 1, 'yaw': 0},
    {'x': 0, 'y':1, 'z': 1.5, 'yaw': 0},
    {'x': 0, 'y':1, 'z': 1, 'yaw': math.pi/2},
    {'x': 0, 'y':0, 'z': 1, 'yaw': -math.pi/2},
]

THRESHOLD = 0.1
WAYPOINT_HOLD_TIME = 1.0

COMMAND = dict(
    cross='takeoff',
    triangle='land',
    square='stop',
    start='control_on',
    select='control_off',
    R1='change_way_point',
    L1='hover',
    L2='no_hover'
    )

class WayPoints(object):
    """
    The Way Points Class manages a list of Way Points
    And changes the reference if the robot has spent
    The required time in the according way point
    """
    def __init__(self, way_points):
        super(WayPoints, self).__init__()
        self._way_points = way_points
        self._in_way_point = False
        self._error = THRESHOLD*10
        self._timer = None
        rospy.loginfo('Next Way Point is %s', self.get_way_point())


    def __len__(self):
        return len(self._way_points)

    def get_way_point(self):
        """Return current way point"""
        if len(self) > 0:
            return self._way_points[0]

    def get_position_error(self, quad_state_msg):
        """Get the position error in the estimation"""
        self._error = 0
        way_point = self.get_way_point()
        if way_point is not None:
            for key, value in way_point.items():
                self._error += abs(value - getattr(quad_state_msg, key))
        if self._error < THRESHOLD and not self._in_way_point:
            self._in_way_point = True
            self._timer = rospy.Timer(rospy.Duration(WAYPOINT_HOLD_TIME),
                                      self.pop, oneshot=True)
        elif self._error > THRESHOLD and self._in_way_point:
            self._in_way_point = False
            self._timer.shutdown()

    def pop(self, dummy_time_event):
        """
        Check if after a given time the robot is still on the way point
        If so, pop the way point and change the reference
        """
        if len(self) > 0:
            self._way_points.pop(0)
            rospy.loginfo('Next Way Point is %s', self.get_way_point())

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
        self._hover = True
        self._controlling = False
        self._joystick = joystick
        self._way_points = WayPoints(WAY_POINTS)
        delta_t = rospy.get_param('processing_time')
        self._publishers = {
            'land': rospy.Publisher('ardrone/land', Empty, queue_size=1),
            'take_off': rospy.Publisher('ardrone/takeoff', Empty, queue_size=1),
            'reset': rospy.Publisher('ardrone/reset', Empty, queue_size=1),
            'cmd': rospy.Publisher('cmd_vel', Twist, queue_size=10./delta_t),
            'closed_loop': rospy.Publisher('ardrone/closed_loop', Bool, queue_size=2),
            'reference': rospy.Publisher('ardrone/reference', QuadrotorState, queue_size=9./delta_t)
            }

        self.control_off()
        rospy.Subscriber('/joy', Joy, callback=self.recieve_joystick)
        rospy.Subscriber('ardrone/estimation', QuadrotorState, callback=self.recieve_estimation)

        rospy.Timer(rospy.Duration(delta_t), self.command_reference)

    def land(self, dummy_arg=None):
        """Land Drone"""
        self.stop()
        self._publishers['land'].publish()

    def takeoff(self, dummy_arg=None):
        """Land Drone"""
        if quadrotor.STATUS[self._reference.get_status()] == quadrotor.LANDED:
            self.stop()
            self._publishers['take_off'].publish()


    def stop(self):
        """Send Drone to Hover Mode"""
        self.hover()
        self.control_off()
        self.command_vel(Twist())
        #rospy.loginfo('Drone to Hover')

    def hover(self):
        """If the Drone is flying take it to hovering"""
        self._hover = True

    def no_hover(self):
        """If the Drone is flying don't let it hover"""
        self._hover = False

    def change_way_point(self):
        """Get the New Way Point"""
        self._way_points.pop(None)

    def command_reference(self, dummy_time_event):
        """Set the reference to the Position Feedback controller"""
        msg = QuadrotorState()
        position = self._reference.get_position()
        msg.x = position[0]
        msg.y = position[1]
        msg.z = position[2]
        msg.yaw = self._reference.get_heading()

        self._publishers['reference'].publish(msg)
        new_reference = self._way_points.get_way_point()
        if new_reference is None:
            self.control_off()
            self.stop()
            rospy.Timer(rospy.Duration(WAYPOINT_HOLD_TIME), self.land, oneshot=True)
        else:
            self._reference.set_position(new_reference['x'], new_reference['y'])
            self._reference.set_altitude(new_reference['z'])
            self._reference.set_heading(new_reference['yaw'])

    def command_vel(self, twist_msg):
        """Send Local velocity to Drone"""
        self._publishers['cmd'].publish(twist_msg)

    def send_control(self):
        """Publish control on or control off"""
        self._publishers['closed_loop'].publish(self._controlling)

    def control_on(self):
        """Set Position Feedback Controller ON"""
        self._controlling = True
        self.send_control()

    def control_off(self):
        """Set Position Feedback Controller ON"""
        self._controlling = False
        self.send_control()

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

        if not self._controlling:
            msg = Twist()
            if not self._hover:
                velocity = getattr(joy_msg, 'axes')
                msg.linear.x = velocity[self._joystick['axes']['R3_UD']]
                msg.linear.y = velocity[self._joystick['axes']['R3_LR']]
                msg.linear.z = velocity[self._joystick['axes']['L3_UD']]
                msg.angular.x = msg.angular.y = 1.0
                msg.angular.z = velocity[self._joystick['axes']['L3_LR']]
            self.command_vel(msg)

    def recieve_estimation(self, state_msg):
        """Recieve quadrotor estimated state"""
        self._reference.set_status(state_msg.status)
        self._way_points.get_position_error(state_msg)

if __name__ == '__main__':
    rospy.init_node('trayectory', anonymous=True)
    TrajectoryCommander('ardrone', joysticks.PS3)
    rospy.spin()
