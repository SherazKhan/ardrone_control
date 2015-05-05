#!/usr/bin/env python
"""
control algorithm for ardrone

"""
import rospy
from ardrone_msgs.msg import QuadrotorState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import ardrone_lib.quadrotor as quadrotor
import ardrone_lib.controllers as controllers
from numpy import array

SATURATION = 1.

class Controller(object):
    """docstring for Controller"""
    def __init__(self, name):
        super(Controller, self).__init__()
        self._estimation = quadrotor.Quadrotor(name)
        self._reference = quadrotor.Quadrotor(name)
        delta_t = rospy.get_param('processing_time')
        self._controllers = {
            'x': controllers.PID(0.1, 0., 0.05*delta_t),
            'y': controllers.PID(0.1, 0., 0.05*delta_t),
            'z': controllers.PID(1.0, 0., 0.0),
            'yaw': controllers.PID(0.8, 0., 0.2*delta_t)
        }

        # self._controllers = {
        #     'x': controllers.Digital([2., 9.7, 5.5], [1., 12.8, 25.], delta_t),
        #     'y': controllers.Digital([3., 14., 5.6], [1., 12.5, 25.], delta_t),
        #     'z': controllers.Digital([1.03], [1.]),
        #     'yaw': controllers.Digital([0.8], [1.])
        # }

        for controller in self._controllers.values():
            controller.set_saturated(False)

        self._publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10./delta_t)
        self._controlling = False
        rospy.Subscriber('ardrone/estimation', QuadrotorState, callback=self.recieve_estimation)
        rospy.Subscriber('ardrone/reference', QuadrotorState, callback=self.recieve_reference)
        rospy.Subscriber('ardrone/closed_loop', Bool, callback=self.recieve_closed_loop)
        rospy.Timer(rospy.Duration(delta_t), self.control)

    def recieve_state_msg(self, quad_obj, msg, method):
        """
        Updates quadrotor object when a QuadrotorState
        msg is recieved
        """
        quad_obj.set_position(msg.x, msg.y)
        quad_obj.set_altitude(msg.z)
        quad_obj.set_heading(msg.yaw)

        for key, controller in self._controllers.items():
            getattr(controller, method)(getattr(msg, key))

    def recieve_estimation(self, msg):
        """
        Recieve State Estimation from sensor fusion node
        Update Estimation locally
        """
        self.recieve_state_msg(self._estimation, msg, 'set_input')

    def recieve_reference(self, msg):
        """
        Recieve Reference from Trajectory Generator
        Update Reference locally
        """
        self.recieve_state_msg(self._estimation, msg, 'set_reference')

    def recieve_closed_loop(self, msg_bool):
        """
        Recieve if publish commands or not
        """
        self._controlling = msg_bool
        for controller in self._controllers.values():
            controller.set_saturated(self._controlling)

    def control(self, time_event):
        """
        Publish previous control step
        Calculate Errors and control output for Next Step
        """
        if time_event.last_real is not None:
            delta_t = float(time_event.current_real.nsecs - time_event.last_real.nsecs)/10**9
            if delta_t > 0 and self._controlling:
                self.publish()

    def _check_saturation(self, msg):
        """
        Check if controller saturate and if so set them in saturation
        """
        if abs(msg.linear.x) > SATURATION:
            self._controllers['x'].set_saturated(True)
        else:
            self._controllers['x'].set_saturated(False)
        if abs(msg.linear.y) > SATURATION:
            self._controllers['y'].set_saturated(True)
        else:
            self._controllers['y'].set_saturated(False)
        if abs(msg.linear.z) > SATURATION:
            self._controllers['z'].set_saturated(True)
        else:
            self._controllers['z'].set_saturated(False)
        if abs(msg.angular.z) > SATURATION:
            self._controllers['yaw'].set_saturated(True)
        else:
            self._controllers['yaw'].set_saturated(False)

    def publish(self):
        """
        Publish Twist Message
        """
        msg = Twist()
        output = self._estimation.global_to_local_rotation(
            array([self._controllers['x'].get_output(),
                   self._controllers['y'].get_output(),
                   0]))

        msg.linear.x = output[0]
        msg.linear.y = output[1]
        msg.linear.z = self._controllers['z'].get_output()
        msg.angular.z = self._controllers['yaw'].get_output()
        self._publisher.publish(msg)
        self._check_saturation(msg)


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    Controller('ardrone')
    rospy.spin()
