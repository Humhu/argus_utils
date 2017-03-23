"""Checks to describe bounds or conditions on runtime parameters.
"""
import math
import numpy as np

class IntegerValued(object):
    """Forces numerical values to be integer-valued based on a
    specified rounding mode.
    """

    def __init__(self, mode='closest'):
        if mode == 'floor':
            self._rounder = math.floor
            self._desc = 'Round floor to integer'
        elif mode == 'ceil':
            self._rounder = math.ceil
            self._desc = 'Round ceil to integer'
        elif mode == 'closest':
            self._rounder = round
            self._desc = 'Round closest to integer'
        else:
            raise ValueError('Unknown rounding mode %s' % mode)

    def __call__(self, v):
        return self._rounder(v)

    def __repr__(self):
        return self._desc

class GreaterThan(object):
    """Forces numerical values to be strictly greater than a lower bound.
    """
    def __init__(self, lim):
        self._desc = 'Strictly greater than %f' % lim
        self._lim = np.nextafter(float(lim), float('inf'))

    def __call__(self, v):
        return max(self._lim, v)

    def __repr__(self):
        return self._desc

class GreaterThanOrEqual(object):
    """Forces numerical values to be greater than or equal to a lower bound.
    """
    def __init__(self, lim):
        self._desc = 'Greater than or equal to %f' % lim
        self._lim = float(lim)

    def __call__(self, v):
        return max(self._lim, v)

    def __repr__(self):
        return self._desc

class LessThan(object):
    """Forces numerical values to be strictly less than a lower bound.
    """
    def __init__(self, lim):
        self._desc = 'Strictly less than %f' % lim
        self._lim = np.nextafter(float(lim), float('-inf'))

    def __call__(self, v):
        return min(self._lim, v)

    def __repr__(self):
        return self._desc

class LessThanOrEqual(object):
    """Forces numerical values to be less than or equal to a lower bound.
    """
    def __init__(self, lim):
        self._desc = 'Less than or equal to %f' % lim
        self._lim = float(lim)

    def __call__(self, v):
        return min(self._lim, v)

    def __repr__(self):
        return self._desc