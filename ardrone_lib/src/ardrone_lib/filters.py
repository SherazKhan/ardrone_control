#!/usr/bin/env python
"""
Various Filters Implementation
"""
from collections import deque
import scipy


class Delay(object):
    """
    A dead-time delay Implemenation
    """
    def __init__(self, time_delay, initial_condition=0.):
        super(Delay, self).__init__()
        n_delays = int(time_delay)+1
        self._input = deque([initial_condition]*n_delays, maxlen=n_delays)

    def __str__(self):
        return 'delays: ' + str(len(self._input))

    def set_input(self, new_input):
        """Input a new value"""
        self._input.append(new_input)

    def get_output(self):
        """Get the system output"""
        return self._input[0]

class TransferFunction(object):
    """
    A digital filter transfer function implemenation using a Direct Form.
    If dt is set then the TF is considered continuous and is discretized
    """
    def __init__(self, num, den, dt=None):
        super(TransferFunction, self).__init__()
        if dt is not None and len(den) > 1:
            lti_sys = scipy.signal.tf2ss(num, den)
            a_k, b_k, c_k, d_k, dt = scipy.signal.cont2discrete(lti_sys, dt)
            num, den = scipy.signal.ss2tf(a_k, b_k, c_k, d_k)
            num = num.reshape(-1,).tolist()
            den = den.reshape(-1,).tolist()
        self._num = num
        self._den = den
        self._input = deque([0]*len(num), maxlen=len(num))
        self._output = deque([0]*len(den), maxlen=len(den))

    def __str__(self):
        return 'num: ' + str(self._num) + ' den: ' + str(self._den)
    def __len__(self):
        return len(self._den)

    def set_input(self, new_input):
        """Input a new value"""
        self._input.append(new_input)
        self._calculate_output()

    def _calculate_output(self):
        """Calculate the system output"""
        new_output = 0
        for idx in range(len(self._num)):
            new_output += self._num[idx] * self._input[idx]
        for idx in range(1, len(self._den)):
            new_output -= self._den[idx] * self._output[idx]
        self._output.append(new_output)

    def get_output(self):
        """Get the system output"""
        return self._output[-1]

    def get_num(self):
        """Return numerator"""
        return self._num

    def get_den(self):
        """Return denominator"""
        return self._den

class TransferFunctionWithDelay(object):
    """
    Implementation of a Transfer Function with dead time delays
    """
    def __init__(self, num, den, time_delays, dt=None):
        super(TransferFunctionWithDelay, self).__init__()
        self._transfer_function = TransferFunction(num, den, dt)
        self._delay = Delay(time_delays)

    def set_input(self, new_input):
        """Input a new value"""
        self._delay.set_input(new_input)
        self._transfer_function.set_input(self._delay.get_output())

    def get_output(self):
        """Get the system output"""
        return self._transfer_function.get_output()

class LeakyIntegrator(object):
    """docstring for LeakyIntegrator"""
    def __init__(self, lambda_factor):
        super(LeakyIntegrator, self).__init__()
        self._lambda = float(lambda_factor)
        self._output = 0.

    def set_input(self, new_input):
        """Set Controller input"""
        self._output *= self._lambda
        self._output += (1-self._lambda) * new_input

    def get_output(self):
        """Return Controller output"""
        return self._output

