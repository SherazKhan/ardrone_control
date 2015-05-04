#!/usr/bin/env python
"""
control algorithm for ardrone

"""
import numpy
from ardrone_lib.filters import TransferFunction


class Controller(object):
    """
    General Controller Properties
    """
    def __init__(self):
        super(Controller, self).__init__()
        self._saturated = False
        self._periodic = False
        self._output = 0.0
        self._reference = 0.0
        self._input = 0.0
        self._error = 0.0

    def set_reference(self, new_reference):
        """Set Controller reference"""
        self._reference = new_reference
        self._calculate_error()

    def set_input(self, new_input):
        """Set Controller input"""
        self._input = new_input
        self._calculate_error()

    def get_output(self):
        """Return Controller output"""
        return self._output

    def is_saturated(self):
        """Check if Controller is saturating actuator"""
        return self._saturated

    def set_saturated(self, is_saturated):
        """Set if Controller is saturating actuator"""
        self._saturated = is_saturated

    def is_periodic(self):
        """Check if position is periodic"""
        return self._periodic

    def set_periodic(self, is_periodic):
        """Set if position is periodic"""
        self._periodic = is_periodic

    def _calculate_error(self):
        """calculate new error"""
        self._error = self._reference - self._input
        if self._periodic:
            self._error = numpy.arctan2(numpy.sin(self._error), numpy.cos(self._error))


class PID(Controller):
    """
    PID controller parallel implementation
    """
    K_I = 0
    K_P = 1
    K_D = 2
    K_D2 = 3
    LAMBDA = 0.1
    def __init__(self, Kp, Ki=0., Kd=0., Kd2=0.):
        super(PID, self).__init__()
        self._gains = numpy.array([Ki, Kp, Kd, Kd2])
        self._parallel_error = numpy.zeros((1, 4))[0]

    def _calculate_error(self):
        """Calculate Controller Error"""
        super(PID, self)._calculate_error()
        self._parallel_error[self.K_D] *= self.LAMBDA
        self._parallel_error[self.K_D] += (1-self.LAMBDA)\
                                         *(self._error - self._parallel_error[self.K_P])

        self._parallel_error[self.K_P] = self._error
        if not self._saturated:
            self._parallel_error[self.K_I] += self._error
        self._calculate_output()

    def _calculate_output(self):
        """Calculate Controller output"""
        self._output = numpy.dot(self._gains, self._parallel_error)

class TrajectoryPID(PID):
    """
    Trajectory Following PID controller
    """
    POSITION = 0
    VELOCITY = 1
    ACCELERATION = 2
    def __init__(self, Kp, Ki=0., Kd=0., Kd2=1.):
        super(TrajectoryPID, self).__init__(Kp, Ki, Kd, Kd2)
        self._trajectory_reference = numpy.zeros((1, 3))[0]
        self._trajectory_input = numpy.zeros((1, 3))[0]

    def _update_vector(self, vector, idx, new_value):
        """Updates vector values"""
        vector[idx] = new_value
        self._calculate_error()

    def set_reference(self, position):
        """Set desired position"""
        super(TrajectoryPID, self).set_reference(position)
        self._update_vector(self._trajectory_reference, self.POSITION, self._reference)

    def set_desired_velocity(self, velocity):
        """Set desired velocity"""
        self._update_vector(self._trajectory_reference, self.VELOCITY, velocity)

    def set_desired_acceleration(self, acceleration):
        """Set desired acceleration"""
        self._update_vector(self._trajectory_reference, self.ACCELERATION, acceleration)

    def set_input(self, position):
        """Set measured position"""
        super(TrajectoryPID, self).set_input(position)
        self._update_vector(self._trajectory_input, self.POSITION, self._input)

    def set_measured_velocity(self, velocity):
        """Set measured velocity"""
        self._update_vector(self._trajectory_input, self.VELOCITY, velocity)

    def set_measured_acceleration(self, acceleration):
        """Set measured acceleration"""
        self._update_vector(self._trajectory_input, self.ACCELERATION, acceleration)

    def _calculate_error(self):
        """Calculate Controller Error"""
        self._parallel_error[self.K_P:(self.K_D2+1)] =\
            self._trajectory_reference - self._trajectory_input
        if not self._saturated:
            self._parallel_error[self.K_I] += self._parallel_error[self.K_P]
        self._calculate_output()


class Digital(Controller):
    """
    Digital Controller:
    args are num, den and optionally dt if needed to digitalize with a ZOH
    """
    def __init__(self, num, den, dt=None):
        super(Digital, self).__init__()
        self._tf = TransferFunction(num, den, dt)

    def __len__(self):
        return len(self._tf)

    def _calculate_error(self):
        """Calculate Controller Error"""
        super(Digital, self)._calculate_error()
        self._tf.set_input(self._error)
        self._calculate_output()

    def _calculate_output(self):
        """Calculate Controller output"""
        self._output = self._tf.get_output()
