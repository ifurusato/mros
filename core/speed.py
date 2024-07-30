#}!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-07-29
# modified: 2024-06-08
#

from enum import Enum

from core.util import Util

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Speed(Enum):
    '''
    Provides an enumeration of both ahead (forward) and astern (reverse)
    Chadburn-style speeds, as corresponding to an abstract speed value.

    NOTE: the speed values here are not used for motor control; those would
    the values enumerated by the Chadburn class.

    The default values for astern and ahead proportional power are initially
    set to zero; these must be set from the YAML application configuration
    via the configure() method.
    '''
    #                                                  proportional power
    #                    name                   speed      astern  ahead
    STOP          = ( 1, 'stop',                  0.0,      0.0,   0.0 )
    DEAD_SLOW     = ( 2, 'dead slow',            20.0,      0.0,   0.0 )
    SLOW          = ( 3, 'slow',                 30.0,      0.0,   0.0 )
    ONE_THIRD     = ( 4, 'one third speed',      40.0,      0.0,   0.0 )
    HALF          = ( 5, 'half speed',           50.0,      0.0,   0.0 )
    TWO_THIRDS    = ( 6, 'two thirds speed',     67.0,      0.0,   0.0 )
    THREE_QUARTER = ( 7, 'three quarter speed',  75.0,      0.0,   0.0 )
    FULL          = ( 8, 'full speed',           90.0,      0.0,   0.0 )
    MAXIMUM       = ( 9, 'maximum speed',       100.0,      0.0,   0.0 )

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, velocity, astern, ahead):
        self._name     = name
        self._velocity = velocity
        self._astern   = astern
        self._ahead    = ahead

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def value(self):
        raise Exception('can\'t call value directly.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def velocity(self):
        '''
        Return the velocity corresponding with this Speed.
        '''
        return self._velocity

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def astern(self):
        '''
        Return the proportional power astern for this Speed.
        '''
        return self._astern

    @astern.setter
    def astern(self, astern):
        '''
        Set the proportional power astern for this Speed.
        '''
        self._astern = astern

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def ahead(self):
        '''
        Return the proportional power ahead for this Speed.
        '''
        return self._ahead

    @ahead.setter
    def ahead(self, ahead):
        '''
        Set the proportional power ahead for this Speed.
        '''
        self._ahead = ahead

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __str__(self):
        return 'Speed.{}:{}v={:5.2f};\t{:5.2f}->{:5.2f}.'.format(self.name, (' ' * max(0, (16 - len(self.name)))),
                self._velocity, self._astern, self._ahead)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def from_string(value):
        for s in Speed:
            if value.upper() == s.name:
                return s
        raise NotImplementedError

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_proportional_power(velocity):
        '''
        Returns the proportional (interpolated) power corresponding to the velocity
        argument. This linear interpolates first across the x axis (velocity,
        ranging from -100 to +100), then linear interpolates across the y axis
        (power, ranging from -1.0 to +1.0) to obtain the power value.

        The returned value is limited by the positive and negative range expressed
        in the YAML configuration.
        '''
        _x_range = Speed.xrange(velocity) # returns the bounding two Speeds
        _s0 = _x_range[0]
        _s1 = _x_range[1]
        # percentage of way that v is along _s0.velocity to _s1.velocity is: _s0 + velocity / ( _v1 - _v0 )
        if _s0.velocity == _s1.velocity:
            _pp = Speed.lerp(_s0.ahead , _s1.ahead, 1.0)
            if velocity < 0:
                _pp *= -1
        else:
            if velocity < 0:
                _pc = ((-1 * _s0.velocity) - velocity) / (_s1.velocity - _s0.velocity)
            else:
                _pc = ( velocity - _s0.velocity ) / ( _s1.velocity - _s0.velocity )
            _xt = Speed.lerp(_s0.velocity, _s1.velocity, _pc)
            _pp = Speed.lerp(_s0.ahead , _s1.ahead, _pc)
            if velocity < 0:
                _pp *= -1
        return _pp

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def lerp(v0: float, v1: float, t: float) -> float:
        return (1 - t) * v0 + t * v1

    @staticmethod
    def inv_lerp(a: float, b: float, v: float) -> float:
        return (v - a) / (b - a)

    @staticmethod
    def remap(i_min: float, imax: float, o_min: float, o_max: float, v: float) -> float:
        return lerp(o_min, o_max, inv_lerp(i_min, imax, v))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def xrange(x):
        if abs(x) >= Speed.MAXIMUM.velocity:
            return [ Speed.MAXIMUM, Speed.MAXIMUM ]
        r = []
        if x < 0:
            x *= -1
            for s in Speed:
                # counting up from zero, first time we're less than value use that as our high value
                if x <= s.velocity:
                    r.append(s)
                    break
            for s in reversed(Speed):
                # counting down from max, first time we're greater than v use that as our low value
                if x >= s.velocity:
                    r.append(s)
                    break
        else:
            for s in reversed(Speed):
                if x >= s.velocity:
                    r.append(s)
                    break
            for s in Speed:
                if x <= s.velocity:
                    r.append(s)
                    break
        if len(r) == 1:
            raise Exception('only 1: speed: {}'.format(r))
        return r

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def print_configuration(log):
        log.info('configured speeds:')
        for _speed in Speed:
            log.info('  {}:{}astern: '.format(_speed.name, Util.repeat(' ', 21 - len(_speed.name))) + '{:>5.2f}'.format(_speed.astern) 
                    + '   ahead: {:>5.2f}'.format(_speed.ahead))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def configure(config):
        '''
        A static method that imports astern and ahead values from the provided
        YAML-sourced configuration.

        This imports from 'mros:motor:speed'
        '''
        _entries = config['mros'].get('motor').get('speed')
        _astern_speeds = _entries.get('astern')
        _ahead_speeds  = _entries.get('ahead')
        for _speed in Speed:
            _speed_enum = Speed.from_string(_speed.name)
            _speed_enum.astern = _astern_speeds.get(_speed_enum.name)
            _speed_enum.ahead  = _ahead_speeds.get(_speed_enum.name)

#EOF
