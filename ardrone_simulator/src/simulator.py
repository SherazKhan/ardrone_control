#!/usr/bin/env python
"""
Ar.Drone Simulator
"""
import rospy

from geometry_msgs.msg import Twist, Vector3Stamped
from ardrone_autonomy.msg import Navdata
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu, NavSatFix, Range
from ardrone_msgs.msg import QuadrotorState

import numpy
import tf

from ardrone_lib import quadrotor
from ardrone_lib import sensors
from ardrone_lib import physics

Z_HIGH = 0.95
Z_LOW = 0.2
#Z_VEL_LANDING = -0.5
#Z_VEL_TAKING_OFF = +0.5
SATURATION = 1.

HOVERING_MSG = Twist()
LANDING_MSG = Twist()
LANDING_MSG.linear.z = -0.5
TAKING_OFF_MSG = Twist()
TAKING_OFF_MSG.linear.z = +0.5


class Simulator(object):
    """docstring for Simulator"""
    def __init__(self):
        super(Simulator, self).__init__()
        self._quadrotor = quadrotor.Quadrotor('sim')
        self._sensors = {
            'gyroscope': sensors.Gyroscope(),
            'accelerometer': sensors.Accelerometer(),
            'magnetometer': sensors.Magnetometer(),
            'gps': sensors.GPS(),
            'velocity': sensors.Velocity(),
            'altimeter': sensors.Altimeter(),
            'marg': sensors.Marg()
        }
        pub_time = rospy.get_param('publish_time')
        sim_time = rospy.get_param('sim_time')
        delay = rospy.get_param('delay')

        self._cmd_vel = Twist()
        self._physics = physics.ArDrone(sim_time, delay)

        self._quadrotor.set_battery(100.)
        self._quadrotor.set_status(quadrotor.STATUS.index(quadrotor.LANDED))
        self._sequencer = 0

        queue_size = 10./pub_time
        self._publishers = {
            'navdata': rospy.Publisher('ardrone/navdata', Navdata, queue_size=queue_size),
            'imu': rospy.Publisher('ardrone/imu', Imu, queue_size=queue_size),
            'mag': rospy.Publisher('ardrone/mag', Vector3Stamped, queue_size=queue_size),
            'fix': rospy.Publisher('ardrone/fix', NavSatFix, queue_size=10.),
            'sonar_height': rospy.Publisher('sonar_height', Range, queue_size=queue_size),
            'truth': rospy.Publisher('ardrone/ground_truth', QuadrotorState, queue_size=queue_size)
            }

        rospy.Subscriber('ardrone/land', Empty, callback=self.land)
        rospy.Subscriber('ardrone/takeoff', Empty, callback=self.takeoff)
        rospy.Subscriber('ardrone/reset', Empty, callback=self.reset)
        rospy.Subscriber('cmd_vel', Twist, callback=self.cmd_vel)

        rospy.Timer(rospy.Duration(sim_time), self.predict)
        rospy.Timer(rospy.Duration(pub_time), self.publish)
        rospy.Timer(rospy.Duration(1.0), self._publish_fix)

    def land(self, dummy_arg):
        """Land Quadrotor"""
        if quadrotor.STATUS[self._quadrotor.get_status()] != quadrotor.LANDED\
        and quadrotor.STATUS[self._quadrotor.get_status()] != quadrotor.LANDING:
            rospy.loginfo('LANDING')
            self._quadrotor.set_status(quadrotor.STATUS.index(quadrotor.LANDING))
            self._cmd_vel = LANDING_MSG

    def takeoff(self, dummy_arg):
        """Take Off Quadrotor"""
        if quadrotor.STATUS[self._quadrotor.get_status()] == quadrotor.LANDED:
            rospy.loginfo('TAKING_OFF')
            self._quadrotor.set_status(quadrotor.STATUS.index(quadrotor.TAKING_OFF))
            self._cmd_vel = TAKING_OFF_MSG

    def reset(self, dummy_arg):
        """Reset Quadrotor"""
        self._quadrotor.set_status(quadrotor.STATUS.index(quadrotor.UNKOWN))
        rospy.loginfo('RESET')

    def cmd_vel(self, twist_msg):
        """Parse cmd_vel msg according to state or drone"""
        vel = [twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z,
               twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z]

        if quadrotor.STATUS[self._quadrotor.get_status()] == quadrotor.FLYING:
            self._cmd_vel = twist_msg
            if sum([x**2 for x in vel]) == 0:
                self._quadrotor.set_status(quadrotor.STATUS.index(quadrotor.HOVERING))
                self._cmd_vel = HOVERING_MSG
        elif quadrotor.STATUS[self._quadrotor.get_status()] == quadrotor.HOVERING:
            if sum([x**2 for x in vel]) == 0:
                self._cmd_vel = HOVERING_MSG
            else:
                self._quadrotor.set_status(quadrotor.STATUS.index(quadrotor.FLYING))
                self._cmd_vel = twist_msg
        elif quadrotor.STATUS[self._quadrotor.get_status()] == quadrotor.LANDING:
            self._cmd_vel = LANDING_MSG
        elif quadrotor.STATUS[self._quadrotor.get_status()] == quadrotor.TAKING_OFF:
            self._cmd_vel = TAKING_OFF_MSG
        else:
            self._cmd_vel = HOVERING_MSG

    def _update_dynamics(self):
        """Update the physics of the simulation"""
        if quadrotor.STATUS[self._quadrotor.get_status()] == quadrotor.FLYING\
        or quadrotor.STATUS[self._quadrotor.get_status()] == quadrotor.LANDING\
        or quadrotor.STATUS[self._quadrotor.get_status()] == quadrotor.TAKING_OFF:
            vel = self._physics.get_output()
            self._quadrotor.set_velocity(vel[0], vel[1], vel[2])
            self._quadrotor.set_omega(0., 0., vel[5])
            self._physics.set_input(self._cmd_vel)

    def predict(self, time_event):
        """Update Quadrotor State"""
        if time_event.last_real is not None:
            delta_t = float(time_event.current_real.nsecs - time_event.last_real.nsecs)/10**9
            if delta_t > 0:
                self._quadrotor.odometric_prediction(delta_t)
                self._quadrotor.update_battery(delta_t)
                self._update_dynamics()


        if self._quadrotor.get_altitude() >= Z_HIGH \
        and quadrotor.STATUS[self._quadrotor.get_status()] == quadrotor.TAKING_OFF:
            self._quadrotor.set_status(quadrotor.STATUS.index(quadrotor.HOVERING))
            self._cmd_vel = HOVERING_MSG
            self._quadrotor.set_velocity(0., 0., 0.)
            self._quadrotor.set_omega(0., 0., 0.)

        if self._quadrotor.get_altitude() <= Z_LOW \
        and quadrotor.STATUS[self._quadrotor.get_status()] == quadrotor.LANDING:
            self._quadrotor.set_status(quadrotor.STATUS.index(quadrotor.LANDED))
            #self._cmd_vel = HOVERING_MSG
            self._quadrotor.set_velocity(0., 0., 0.)
            self._quadrotor.set_omega(0., 0., 0.)

        if self._quadrotor.get_battery() < 20:
            self.land(None)
            rospy.loginfo('No more battery')

    def publish(self, time_event):
        """Publish messages"""
        if time_event.last_real is not None:
            delta_t = float(time_event.current_real.nsecs - time_event.last_real.nsecs)/10**9
            if delta_t > 0:
                self._update_sensors()
                self._publish_navdata()
                self._publish_imu()
                self._publish_mag()
                self._publish_altd()
                self._sequencer += 1

    def _update_sensors(self):
        """Update sensor measurements"""
        self._sensors['gyroscope'].set_true_value(self._quadrotor.get_velocity()[3:6])
        self._sensors['accelerometer'].set_orientation(self._quadrotor.get_quaternion())
        self._sensors['magnetometer'].set_orientation(self._quadrotor.get_quaternion())
        self._sensors['gps'].set_position(self._quadrotor.get_position())
        self._sensors['altimeter'].set_true_value(numpy.array([self._quadrotor.get_altitude()]))
        self._sensors['marg'].set_true_value(self._quadrotor.get_quaternion().get_quaternion())
        self._sensors['velocity'].set_true_value(self._quadrotor.get_velocity()[0:2])

    def _stamp_msg(self, msg):
        """Get unstamped msg and return stamped msg"""
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = self._sequencer
        return msg

    def _publish_navdata(self):
        """Publish Navdata"""
        orientation = tf.transformations.euler_from_quaternion(self._sensors['marg'].get_output())
        velocity = self._sensors['velocity'].get_output()
        altitude = self._sensors['altimeter'].get_output()

        msg = self._stamp_msg(Navdata())

        msg.altd = altitude[0] * 1000
        msg.vx = velocity[0] * 1000
        msg.vy = velocity[1] * 1000
        msg.rotX = orientation[0] * 180 / numpy.pi
        msg.rotY = orientation[1] * 180 / numpy.pi
        msg.rotZ = orientation[2] * 180 / numpy.pi
        msg.batteryPercent = self._quadrotor.get_battery()
        msg.state = self._quadrotor.get_status()

        self._publishers['navdata'].publish(msg)

    def _publish_imu(self):
        """Publish IMU messages"""
        orientation = self._sensors['marg'].get_output()
        omega = self._sensors['gyroscope'].get_output()
        acceleration = self._sensors['accelerometer'].get_output()

        msg = self._stamp_msg(Imu())

        msg.orientation.x = orientation[0]
        msg.orientation.y = orientation[1]
        msg.orientation.z = orientation[2]
        msg.orientation.w = orientation[3]

        msg.angular_velocity.x = omega[0]
        msg.angular_velocity.y = omega[1]
        msg.angular_velocity.z = omega[2]

        msg.linear_acceleration.x = acceleration[0]
        msg.linear_acceleration.y = acceleration[1]
        msg.linear_acceleration.z = acceleration[2]

        self._publishers['imu'].publish(msg)

    def _publish_mag(self):
        """Publish Magnetometer messages"""
        mag = self._sensors['magnetometer'].get_output()

        msg = self._stamp_msg(Vector3Stamped())

        msg.vector.x = mag[0]
        msg.vector.y = mag[1]
        msg.vector.z = mag[2]

        self._publishers['mag'].publish(msg)

    def _publish_fix(self, dummy_time):
        """Publish Fix Messages"""
        gps = self._sensors['gps'].get_output()

        msg = self._stamp_msg(NavSatFix())

        msg.status.status = 0
        msg.status.service = 1

        msg.latitude = gps[0]
        msg.longitude = gps[1]
        msg.altitude = gps[2]

        self._publishers['fix'].publish(msg)

    def _publish_altd(self):
        """Publish Altitude"""
        altd = self._sensors['altimeter'].get_output()
        sensor_range = self._sensors['altimeter'].get_range()

        msg = self._stamp_msg(Range())
        msg.range = altd[0] * 1000
        msg.min_range = sensor_range[0]
        msg.min_range = sensor_range[1]

        self._publishers['sonar_height'].publish(msg)

    def _publish_true(self):
        """Publish True State"""
        position = self._quadrotor.get_position()
        heading = self._quadrotor.get_heading()
        msg = self._stamp_msg(QuadrotorState())

        msg.x = position[0]
        msg.y = position[1]
        msg.z = position[2]
        msg.yaw = heading
        self._publishers['truth'].publish(msg)


if __name__ == '__main__':
    rospy.init_node('simulator', anonymous=True)
    Simulator()
    rospy.spin()
