#!/usr/bin/env python
"""
Sensor Implemenation for Simulator
"""
import utm # GPS OBJECT
import geomag #Magnetometer OBJECT
import numpy
import ardrone_lib.quaternion as quaternion

class Sensor(object):
    """Sensor Class that considers bias, noise, cross-axis coupling and
    sensor maximum-minimum range. The noise model used is gaussian"""
    NULL_SENSOR_DATA = {
        'bias': 0.0,
        'noise': 0.0,
        'cross_coupling': 0.0,
        'min': float('-inf'),
        'max': float('+inf')
    }
    def __init__(self, dim, sensor_data=None):
        super(Sensor, self).__init__()
        if sensor_data is None:
            data = self.NULL_SENSOR_DATA
        else:
            data = sensor_data
        self._dim = dim
        self._bias = data['bias']*numpy.random.normal(0, 0.33, (1, dim))[0]
        self._noise = data['noise']*numpy.ones((1, dim))[0]
        self._true = numpy.zeros((1, dim))[0]
        self._cross_coupling = data['cross_coupling']*(
            numpy.ones((dim, dim)) - numpy.identity(dim))
        self._range = (data['min'], data['max'])
        self._measurement = numpy.zeros((1, dim))[0]
        self._input = numpy.zeros((1, dim))[0]

    def set_true_value(self, true_val):
        """Set true value"""
        if type(true_val) is not type(self._true):
            raise TypeError
        self._true = true_val

    def get_simulation(self):
        """Simulate bias, noise, cross_coupling and saturation"""
        if (self._noise == numpy.zeros((1, self._dim))).all():
            output = self.get_output() + self._bias
        else:
            output = self.get_output() + numpy.random.normal(self._bias, self._noise)[0]
        self._bias *= numpy.random.normal(1., 0.033, (1, len(self._bias)))[0]
        output += numpy.dot(self._cross_coupling, output)
        for idx in range(len(output)):
            if output[idx] > self._range[1]:
                output[idx] = self._range[1]
            elif output[idx] < self._range[0]:
                output[idx] = self._range[0]
        return output

    def get_output(self):
        """Return true value"""
        return self._true

    def get_range(self):
        """Return sensor range"""
        return self._range

    def set_measurement(self, measured_output):
        """Set measured"""
        self._measurement = measured_output
        self._calculate_input()

    def _calculate_input(self):
        """calculate the input that produce the given measurement"""
        self._input = self._measurement

    def get_input(self):
        """Given a measured output, return value that would measure_it"""
        return self._input

class Gyroscope(Sensor, object):
    """Gyroscope Sensor simulated Class"""
    def __init__(self, sensor_data=None):
        super(Gyroscope, self).__init__(3, sensor_data)

class Velocity(Sensor, object):
    """Velocity Sensor simulated Class. Actually, this is obtained by
    fusing accelerometer and a Lucas-Kande Tracker"""
    def __init__(self, sensor_data=None):
        super(Velocity, self).__init__(2, sensor_data)

class Accelerometer(Sensor, object):
    """Accelerometer Sensor simulated Class"""
    GRAVITY = 9.81
    def __init__(self, sensor_data=None):
        super(Accelerometer, self).__init__(3, sensor_data)
        self._magnetic_dec = geomag.geomag.GeoMag()
        self._input = numpy.zeros((1, 2))[0]

    def set_orientation(self, true_quat, body_acc=numpy.zeros((1, 3))[0]):
        """Project gravity into Body Axes"""
        acceleration = (true_quat
                        *quaternion.Quaternion(0., 0., self.GRAVITY, 0.)
                        *true_quat.conjugated()
                       ).get_quaternion()[0:3] + body_acc
        self.set_true_value(acceleration)

    def _calculate_input(self):
        """Calculate roll and pitch angles"""
        self._input[0] = numpy.arctan2(self._measurement[1], self._measurement[2])
        self._input[1] = numpy.arctan(
            -self._measurement[0]/numpy.sqrt(self._measurement[1]**2 + self._measurement[2]**2)
            )

    def get_roll(self):
        """return roll angle"""
        return self._input[0]

    def get_pitch(self):
        """return pitch angle"""
        return self._input[1]

class Magnetometer(Sensor, object):
    """Magnetometer Sensor simulated Class"""
    gmag = geomag.geomag.GeoMag()
    def __init__(self, sensor_data=None, lat=-34.603530, lon=-58.367697):
        super(Magnetometer, self).__init__(3, sensor_data)
        self._mag = self.gmag.GeoMag(lat, lon)
        self._input = numpy.zeros((1, 1))[0]
        self._hard_iron = numpy.zeros((1, 3))[0]
        self._roll = 0.0
        self._pitch = 0.0

    def get_output(self):
        """return output modified by true heading"""
        true_out = super(Magnetometer, self).get_output()
        mag_out = true_out - self._mag.dec * numpy.pi/180
        return numpy.arctan2(numpy.sin(mag_out), numpy.cos(mag_out))

    def set_orientation(self, true_quat):
        """Project earth magnetic field into Body Axes"""
        magnetometer = (true_quat.conjugated()
                        *quaternion.Quaternion(self._mag.bx/1000,
                                               -self._mag.by/1000,
                                               -self._mag.bz/1000, 0.)
                        *true_quat
                       ).get_quaternion()[0:3]
        self.set_true_value(magnetometer)

    def set_latlon(self, lat, lon):
        """Set latlon to change magnetic declination"""
        self._mag = self.gmag.GeoMag(lat, lon)

    def set_attitude(self, roll, pitch):
        """set roll and pitch angles for heading correction"""
        self._roll = roll
        self._pitch = pitch

    def set_hard_iron(self, v_x, v_y, v_z):
        """set measured hard iron"""
        self._hard_iron = numpy.array([v_x, v_y, v_z])

    def _calculate_input(self):
        """calculate the heading"""
        field = self._measurement - self._hard_iron
        bfy = field*numpy.sin(self._roll)\
             -field*numpy.cos(self._roll)
        bfx = field*numpy.cos(self._pitch)\
             +field*numpy.sin(self._pitch)*numpy.sin(self._roll)\
             +field*numpy.sin(self._pitch)*numpy.cos(self._roll)

        self._input[0] = numpy.arctan2(-bfy, bfx) + self._mag.dec * numpy.pi/180.0

    def get_yaw(self):
        """return yaw angle"""
        return self._input[0]

class Marg(Sensor, object):
    """Marg Sensor simulated Class. It does a quaternion orientation estimation"""
    def __init__(self, sensor_data=None):
        super(Marg, self).__init__(4, sensor_data)

    def get_output(self):
        """Returned normalized output"""
        output = super(Marg, self).get_output()
        return output/numpy.linalg.norm(output)

class GPS(Sensor, object):
    """
    GPS Sensor simulated Class
    """
    def __init__(self, lat=-34.603530, lon=-58.367697, alt=0., sensor_data=None):
        super(GPS, self).__init__(3, sensor_data)
        self._initial_utm = utm.from_latlon(lat, lon)
        self._initial_alt = alt
        self.set_true_value(numpy.array([lat, lon, alt]))

    def set_position(self, true_position):
        """
        Set True Position and update LAT, LON, ALT
        """
        lat, lon = utm.to_latlon(
            self._initial_utm[0] + true_position[0],
            self._initial_utm[1] + true_position[1],
            self._initial_utm[2],
            self._initial_utm[3])
        self.set_true_value(numpy.array([lat, lon, true_position[2]]))

    def _calculate_input(self):
        """Given a measured lat, lon, alt, return dx, dy, dz"""
        actual_utm = utm.from_latlon(self._measurement[0], self._measurement[1])
        self._input[0] = actual_utm[0] - self._initial_utm[0]
        self._input[1] = actual_utm[1] - self._initial_utm[1]
        self._input[2] = self._measurement[2] - self._initial_alt

class Altimeter(Sensor, object):
    """Sonar Altimeter Sensor simulated Class"""
    def __init__(self, sensor_data=None):
        super(Altimeter, self).__init__(1, sensor_data)

if __name__ == '__main__':
    from ardrone_lib.test.test_sensors import TestSensors
    import unittest
    unittest.main()
