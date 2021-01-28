import math
import logging
from time import time
from collections import deque, namedtuple


# Based on Arduino PID Library
# See https://github.com/br3ttb/Arduino-PID-Library
class PIDController(object):
    """A proportional-integral-derivative controller.

    Args:
        sampletime (float): The interval between calc() calls.
        kp (float): Proportional coefficient.
        ki (float): Integral coefficient.
        kd (float): Derivative coefficient.
        out_min (float): Lower output limit.
        out_max (float): Upper output limit.
        time (function): A function which returns the current time in seconds.
    """

    def __init__(self, sampletime, kp, ki, kd, out_min=float('-inf'),
                 out_max=float('inf'), time=time):

        if sampletime <= 0:
            raise ValueError('sampletime must be greater than 0')
        if out_min >= out_max:
            raise ValueError('out_min must be less than out_max')

        self._logger = logging.getLogger(type(self).__name__)
        self._sampletime = sampletime
        self._sampletime_ms = sampletime * 1000
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._out_min = out_min
        self._out_max = out_max
        self._integral = 0
        self._last_input = 0
        self._last_output = 0
        self._last_calc_timestamp = 0
        self._time = time

    @property
    def kp(self):
        """Return the PID Controller Kp value."""
        return self._Kp

    @kp.setter
    def kp(self, value):
        """Set new PID Controller Kp value."""
        if value is None:
            raise ValueError('kp must be specified')
        self._Kp = value

    @property
    def ki(self):
        """Return the PID Controller Ki value."""
        return self._Ki / self._sampletime

    @ki.setter
    def ki(self, value):
        """Set new PID Controller Ki value."""
        if value is None:
            raise ValueError('ki must be specified')
        self._Ki = value * self._sampletime

    @property
    def kd(self):
        """Return the PID Controller Kd value."""
        return self._Kd * self._sampletime

    @kd.setter
    def kd(self, value):
        """Set new PID Controller Kd value."""
        if value is None:
            raise ValueError('kd must be specified')
        self._Kd = value / self._sampletime

    def calc(self, input_val, setpoint):
        """Adjusts and holds the given setpoint.

        Args:
            input_val (float): The input value.
            setpoint (float): The target value.

        Returns:
            A value between `out_min` and `out_max`.
        """
        now = self._time() * 1000

        if (now - self._last_calc_timestamp) < self._sampletime_ms:
            return self._last_output

        # Compute all the working error variables
        error = setpoint - input_val
        input_diff = input_val - self._last_input

        # In order to prevent windup, only integrate if the process is not saturated
        if self._last_output < self._out_max and self._last_output > self._out_min:
            self._integral += self._Ki * error
            self._integral = min(self._integral, self._out_max)
            self._integral = max(self._integral, self._out_min)

        p = self._Kp * error
        i = self._integral
        d = -(self._Kd * input_diff)

        # Compute PID Output
        self._last_output = p + i + d
        self._last_output = min(self._last_output, self._out_max)
        self._last_output = max(self._last_output, self._out_min)

        # Log some debug info
        self._logger.debug('P: {0}'.format(p))
        self._logger.debug('I: {0}'.format(i))
        self._logger.debug('D: {0}'.format(d))
        self._logger.debug('output: {0}'.format(self._last_output))

        # Remember some variables for next time
        self._last_input = input_val
        self._last_calc_timestamp = now
        return self._last_output
