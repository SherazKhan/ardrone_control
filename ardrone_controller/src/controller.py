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
        delta_t = rospy.get_param('processing_time', 0.1)
        #self._controllers = {
        #    'x': controllers.PID(0.1, 0., 0.05*delta_t),
        #    'y': controllers.PID(0.1, 0., 0.05*delta_t),
        #    'z': controllers.PID(1.0, 0., 0.0),
        #    'yaw': controllers.PID(0.8, 0., 0.2*delta_t)
        #}
        self._controllers = {
            'x': controllers.Digital([2., 9.7, 5.5], [1., 12.8, 25.], delta_t),
            'y': controllers.Digital([3., 14., 5.6], [1., 12.5, 25.], delta_t),
            'z': controllers.Digital([1.03], [1.]),
            'yaw': controllers.PID(0.8, 0., 0.2*delta_t)
        }

        for controller in self._controllers.values():
            controller.set_saturated(False)

        self._publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10./delta_t)
        self._controlling = False
        rospy.Subscriber('ardrone/estimation', QuadrotorState, callback=self.recieve_estimation)
        rospy.Subscriber('ardrone/reference', QuadrotorState, callback=self.recieve_reference)
        rospy.Subscriber('ardrone/closed_loop', Bool, callback=self.recieve_closed_loop)
        rospy.Timer(rospy.Duration(delta_t), self.control)

    def _recieve_state_msg(self, quad_obj, msg, method):
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
        """Recieve State Estimation from sensor fusion node"""
        if self._controlling:
            self._recieve_state_msg(self._estimation, msg, 'set_input')
        self._estimation.set_status(msg.status)

    def recieve_reference(self, msg):
        """Recieve Reference from Trajectory Generator"""
        if self._controlling:
            self._recieve_state_msg(self._estimation, msg, 'set_reference')

    def recieve_closed_loop(self, msg_bool):
        """Recieve if publish commands or not"""
        self._controlling = msg_bool
        for controller in self._controllers.values():
            controller.set_saturated(self._controlling)

    def control(self, time_event):
        """Calculate Errors and control output for Next Step"""
        if time_event.last_real is not None:
            delta_t = float(time_event.current_real.nsecs - time_event.last_real.nsecs)/10**9
            if delta_t > 0 and self._controlling:
                for controller in self._controllers.values():
                    controller.calculate_error()
                self.publish()

    def _check_saturation(self):
        """Check if controller saturate and if so set them in saturation"""
        for controller in self._controllers.values():
            if abs(controller.get_output()) > SATURATION:
                controller.set_saturated(True)
            else:
                controller.set_saturated(False)

    def publish(self):
        """Publish Twist Message"""
        msg = Twist()
        output = self._estimation.global_to_local_rotation(
            array([self._controllers['x'].get_output(),
                   self._controllers['y'].get_output(),
                   0]))

        msg.linear.x = output[0]
        msg.linear.y = output[1]
        msg.linear.z = self._controllers['z'].get_output()
        msg.angular.z = self._controllers['yaw'].get_output()
        self._check_saturation()
        self._publisher.publish(msg)



if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    Controller('ardrone')
    rospy.spin()
