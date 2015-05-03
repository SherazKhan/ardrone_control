"""
Quadrotor State Class definition for multiple usage
Properties are:
    global position
    local velocity
    heading
    altitude
    status
    battery percentage
Methods:
    global_to_local_rotation
        rotates global vector
"""

from numpy import array, dot
import ardrone_lib.quaternion as quaternion
import tf
UNKOWN = 'Unkown'
INITED = 'Inited'
LANDED = 'Landed'
FLYING = 'Flying'
HOVERING = 'Hovering'
TEST = 'Test'
TAKING_OFF = 'Taking Off'
LANDING = 'Landing'
LOOPING = 'Looping'


STATUS = [
    UNKOWN,
    INITED,
    LANDED,
    FLYING,
    HOVERING,
    TEST,
    TAKING_OFF,
    FLYING,
    LANDING,
    LOOPING
    ]

BATTERY_LIFE = 12*60 #12 minutes

class Quadrotor(object):
    """
    State Class for a Quadrotor
    """
    def __init__(self, name):
        super(Quadrotor, self).__init__()
        self._name = name
        self._global_position = array([0., 0., 0.]) #x, y, z
        self._local_velocity = array([0., 0., 0., 0., 0., 0.]) #vx, vy, vz, wx, wy, wz
        self._rotation = array([0., 0., 0.]) #roll, pitch, yaw
        self._quaternion = quaternion.Quaternion(0., 0., 0., 1.) #qx, qy, qz, qw
        #self._local_to_global = array([[1., 0.], [0., 1.]])
        self._status = 0
        self._battery = 0.0

    def global_to_local_rotation(self, vector):
        """
        Return matrix to transform a global vector into a local vector
        """
        roll, pitch, yaw = self._rotation
        global_to_local = tf.transformations.euler_matrix(roll, pitch, yaw).T
        return dot(global_to_local[0:3, 0:3], vector)

    def local_to_global_rotation(self, vector):
        """
        Return matrix to transform a local vector into a global vector
        """
        roll, pitch, yaw = self._rotation
        local_to_global = tf.transformations.euler_matrix(roll, pitch, yaw)
        return dot(local_to_global[0:3, 0:3], vector)

    def odometric_prediction(self, delta_t):
        """
        given a delta_t calculate the next global position
        """
        self._global_position += self.local_to_global_rotation(self._local_velocity[0:3]) * delta_t
        omega_x, omega_y, omega_z = self._local_velocity[3:]
        self._quaternion.update(omega_x, omega_y, omega_z, delta_t)
        roll, pitch, yaw = self._quaternion.get_euler()
        self._rotation[0] = roll
        self._rotation[1] = pitch
        self._rotation[2] = yaw
        if self._status == STATUS.index(LANDED)\
        or self._status == STATUS.index(UNKOWN)\
        or self._status == STATUS.index(INITED):
            self._battery -= 1.* delta_t / BATTERY_LIFE
        else:
            self._battery -= 100.* delta_t / BATTERY_LIFE

    def get_name(self):
        """
        Get Quadrotor's name
        """
        return self._name

    def set_position(self, pos_x, pos_y, pos_z=0.):
        """
        Set Quadrotor's global position
        """
        self._global_position[0] = pos_x
        self._global_position[1] = pos_y
        self._global_position[2] = pos_z

    def get_position(self):
        """
        Get Quadrotor's global position
        """
        return self._global_position

    def set_altitude(self, altitude):
        """
        Set Quadrotor's altitude
        """
        self._global_position[2] = altitude

    def get_altitude(self):
        """
        Get Quadrotor's altitude
        """
        return self._global_position[2]

    def set_velocity(self, vel_x, vel_y, vel_z=0., ):
        """
        Set Quadrotor's local velocity
        """
        self._local_velocity[0] = vel_x
        self._local_velocity[1] = vel_y
        self._local_velocity[2] = vel_z

    def set_omega(self, omega_x, omega_y, omega_z):
        """
        Set Quadrotor's local rotational velocity
        """
        self._local_velocity[3] = omega_x
        self._local_velocity[4] = omega_y
        self._local_velocity[5] = omega_z

    def get_velocity(self):
        """
        Get Quadrotor's local velocity
        """
        return self._local_velocity

    def set_heading(self, yaw):
        """
        Set Quadrotor's heading
        """
        self._rotation[2] = yaw
        self._quaternion.set_euler(self._rotation[0], self._rotation[1], self._rotation[2])

    def get_heading(self):
        """
        Get Quadrotor's heading
        """
        return self._rotation[2]

    def set_rotation(self, roll, pitch, yaw):
        """
        Set Quadrotor's Orientation using
        ROLL, PITCH, YAW convention
        """
        self._rotation[0] = roll
        self._rotation[1] = pitch
        self._rotation[2] = yaw
        self._quaternion.set_euler(self._rotation[0], self._rotation[1], self._rotation[2])

    def get_rotation(self):
        """
        Get Quadrotor's Orientation using
        ROLL, PITCH, YAW convention
        """
        return self._rotation

    def get_quaternion(self):
        """
        Get Quadrotor's Orientation in Quaternion form
        """
        return self._quaternion

    def set_status(self, idx):
        """
        Set Quadrotor's Status
        """
        self._status = idx

    def get_status(self):
        """
        Get Quadrotor's Status
        """
        return self._status

    def set_battery(self, battery):
        """
        Set Quadrotor battery
        """
        self._battery = battery

    def get_battery(self):
        """
        Get Quadrotor battery
        """
        return self._battery

if __name__ == '__main__':
    QUAD = Quadrotor('a')
    print QUAD.get_name()
    QUAD.set_velocity(1.0, 0.5, 0.1)
    QUAD.set_omega(0.0, 0.0, 0.1)
    for i in range(10):
        QUAD.odometric_prediction(0.1)

    print QUAD.get_position()
    print QUAD.get_heading()
