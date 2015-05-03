#!/usr/bin/env python
"""
sensor fusion algorithm for ardrone
"""
import rospy
from ardrone_autonomy.msg import Navdata
from ardrone_msgs.msg import QuadrotorState
import math

import ardrone_lib.quadrotor as quadrotor


class SensorFusion(object):
    """
    Sensor Fusion Class:
    Reads navdata
    Publishes odometry
    """
    def __init__(self, name):
        super(SensorFusion, self).__init__()
        self._quadrotor = quadrotor.Quadrotor(name)
        delta_t = rospy.get_param('processing_time')
        self._sequencer = 0
        self._publisher = rospy.Publisher('ardrone/estimation', QuadrotorState,\
            queue_size=10./delta_t)
        rospy.Subscriber('ardrone/navdata', Navdata, callback=self.recieve_navdata)
        rospy.Timer(rospy.Duration(delta_t), self.predict)

    def recieve_navdata(self, navdata):
        """
        Callback from navdata from Ar.Drone
        Reads state, battery percentage
        """
        # convert to originally sent drone values (undo ardrone_autonomy changes)
        navdata.ax *= -1 # ax inverted
        navdata.az *= -1 # az inverted

        self._quadrotor.set_velocity(navdata.vx / 1000.0, navdata.vy / 1000.0, 0)
        self._quadrotor.set_heading(navdata.rotZ * math.pi / 180.0)
        self._quadrotor.set_altitude(navdata.altd / 1000.0)
        self._quadrotor.set_status(navdata.state)
        self._quadrotor.set_battery(navdata.batteryPercent)
        self.publish()

    def recieve_fix(self, fix):
        """
        Callback from gps
        Reads latitude, longitude, altitude
        Correct Prediction
        """
        pass

    def recieve_imu(self, imu_msg):
        """
        Callback from IMU
        Reads gyroscope and accelerometer
        Correct Pitch and Roll with Accelerometer
        """
        self._quadrotor.set_omega(
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
            )

    def recieve_mag(self, mag):
        """
        Callback from magnetometer
        Reads magnetometer
        Correct Magnetometer
        """
        pass

    def publish(self):
        """
        Publish Quadrotor Position
        """
        msg = QuadrotorState()
        position = self._quadrotor.get_position()
        msg.x = position[0]
        msg.y = position[1]
        msg.z = position[2]
        msg.yaw = self._quadrotor.get_heading()
        msg.battery = self._quadrotor.get_battery()
        msg.status = self._quadrotor.get_status()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._quadrotor.get_name()
        msg.header.seq = self._sequencer
        self._publisher.publish(msg)

    def predict(self, time_event):
        """
        Predict Quadrotor State
        Timer Event Callback
        """
        if time_event.last_real is not None:
            delta_t = float(time_event.current_real.nsecs - time_event.last_real.nsecs)/10**9
            if delta_t > 0:
                self._quadrotor.odometric_prediction(delta_t)
                self.publish()
                self._sequencer += 1

if __name__ == '__main__':
    rospy.init_node('sensor_fusion', anonymous=True)
    SensorFusion(name='ardrone')
    rospy.spin()


