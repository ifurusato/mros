#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# author:   Jon Hylands for the calculate_steering() function (thank you!)
# created:  2024-05-22
# modified: 2024-06-03
#

import sys, time
from threading import Thread
from enum import Enum
import asyncio, itertools, random, traceback
from math import isclose
from datetime import datetime as dt
import math

from colorama import init, Fore, Style
init()

import core.globals as globals
globals.init()

from core.logger import Logger, Level
from core.component import Component
from core.event import Event
from core.orientation import Orientation
from core.rate import Rate
from core.message import Message, Payload
from hardware.servo import Servo
from hardware.steering_mode import SteeringMode
from hardware.slew import SlewRate

USE_THREADING = False

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class ServoController(Component):
    '''
    A servo controller that asbtracts the actual control of servos from
    the physical servo control hardware.

    This relies upon both a SlewLimiter and JerkLimiter so that velocity
    and power (resp.) changes occur gradually and safely.

    :param config:          the application configuration
    :param motor_config:    the MotorConfigurator object
    :param external_clock:  optional external system clock
    :param level:           the logging level
    '''
    ROTATION_DELAY_MS = 50

    def __init__(self, config, level=Level.INFO):
        self._log = Logger('motor-ctrl', level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._log.info('initialising servos…')
        if not isinstance(config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(name)))
        self._is_daemon = True
        self._half_length  = config.get('mros').get('geometry').get('wheel_base') / 2
        self._half_width   = config.get('mros').get('geometry').get('wheel_track') / 2
        self._wheel_offset = config.get('mros').get('geometry').get('wheel_offset')
        _cfg = config.get('mros').get('servo_controller')
        self._rotate_angle = _cfg.get('rotate_mode') # angle for rotate mode
        self._pfwd_servo = Servo(_cfg, Orientation.PFWD, True, level)
        self._sfwd_servo = Servo(_cfg, Orientation.SFWD, True, level)
        self._pmid_servo = None # Servo(_cfg, Orientation.PMID, True, level)
        self._smid_servo = None # Servo(_cfg, Orientation.SMID, True, level)
        self._paft_servo = Servo(_cfg, Orientation.PAFT, True, level)
        self._saft_servo = Servo(_cfg, Orientation.SAFT, True, level)
        self._all_servos = self._get_servos()
        # default mode
        self.set_mode(SteeringMode.SKID)
        self._log.info('servo controller ready with {} servos.'.format(len(self._all_servos)))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return 'servo-ctrl'

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_servos(self):
        '''
        Returns a list containing all instantiated servos.
        This includes only instantiated servos.
        '''
        return self._all_servos

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_servos(self):
        _list = []
        if self._pfwd_servo:
            _list.append(self._pfwd_servo)
        if self._sfwd_servo:
            _list.append(self._sfwd_servo)
        if self._pmid_servo:
            _list.append(self._pmid_servo)
        if self._smid_servo:
            _list.append(self._smid_servo)
        if self._paft_servo:
            _list.append(self._paft_servo)
        if self._saft_servo:
            _list.append(self._saft_servo)
        return _list

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_servo(self, orientation):
        '''
        Returns the servo with the matching orientation.
        '''
        if orientation is Orientation.PFWD:
            return self._pfwd_servo
        elif orientation is Orientation.SFWD:
            return self._sfwd_servo
        elif orientation is Orientation.PMID:
            return self._pmid_servo
        elif orientation is Orientation.SMID:
            return self._smid_servo
        elif orientation is Orientation.PAFT:
            return self._paft_servo
        elif orientation is Orientation.SAFT:
            return self._saft_servo
        else:
            raise Exception('unsupported orientation.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_mode(self, steering_mode):
        '''
        Sets the angles of the servos according to the steering mode. These
        are all fixed angles.
        '''
        if steering_mode is SteeringMode.ACKERMANN:
            raise Exception('unsupported steering mode: {}'.format(steering_mode.name))
        elif steering_mode is SteeringMode.SKID:
           self.recenter()
        elif steering_mode is SteeringMode.AFRS:
            raise Exception('unsupported steering mode: {}'.format(steering_mode.name))
        elif steering_mode is SteeringMode.OMNIDIRECTIONAL:
            raise Exception('unsupported steering mode: {}'.format(steering_mode.name))
        elif steering_mode is SteeringMode.SIDEWAYS:
            raise Exception('unsupported steering mode: {}'.format(steering_mode.name))
        elif steering_mode is SteeringMode.ROTATE:
            self._set_rotate_mode()
        elif steering_mode is SteeringMode.FORWARD_PIVOT:
            raise Exception('unsupported steering mode: {}'.format(steering_mode.name))
        elif steering_mode is SteeringMode.AFT_PIVOT:
            raise Exception('unsupported steering mode: {}'.format(steering_mode.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_ackermann_angle(self, angle):
        raise Exception('unsupported steering mode: ackermann')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _set_rotate(self, servo, angle):
        _current_angle = int(servo.angle)
        if _current_angle > angle:
            self._log.info('set rotate for servo {} from {:d}° DOWN to {:d}°'.format(servo.name, _current_angle, angle) + Style.RESET_ALL)
            for a in range(_current_angle, angle, -1):
                servo.set_angle(a)
                time.sleep(ServoController.ROTATION_DELAY_MS / 1000)
        elif _current_angle < angle:
            self._log.info('set rotate for servo {} from {:d}° UP to {:d}°'.format(servo.name, _current_angle, angle) + Style.RESET_ALL)
            for a in range(_current_angle, angle):
                servo.set_angle(a)
                time.sleep(ServoController.ROTATION_DELAY_MS / 1000)
        self._log.info('servo {} rotate complete.'.format(servo.name) + Style.RESET_ALL)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _set_rotate_mode(self):
        if USE_THREADING:
            _t_pfwd = Thread(target = self._set_rotate, args=[self._pfwd_servo, -1 * int(self._rotate_angle)], name='rotate_pfwd', daemon=self._is_daemon)
            _t_sfwd = Thread(target = self._set_rotate, args=[self._sfwd_servo, int(self._rotate_angle)], name='rotate_sfwd', daemon=self._is_daemon)
#           _t_pmid = Thread(target = self._set_rotate, args=[self._pmid_servo, 0], name='rotate_pmid', daemon=self._is_daemon)
#           _t_smid = Thread(target = self._set_rotate, args=[self._smid_servo, 0], name='rotate_smid', daemon=self._is_daemon)
            _t_paft = Thread(target = self._set_rotate, args=[self._paft_servo, int(self._rotate_angle)], name='rotate_paft', daemon=self._is_daemon)
            _t_saft = Thread(target = self._set_rotate, args=[self._saft_servo, -1 * int(self._rotate_angle)], name='rotate_saft', daemon=self._is_daemon)
            _t_pfwd.start()
            _t_sfwd.start()
#           _t_pmid.start()
#           _t_smid.start()
            _t_paft.start()
            _t_saft.start()
        else:
            self._pfwd_servo.set_angle(-1 * self._rotate_angle)
            self._sfwd_servo.set_angle(self._rotate_angle)
#           self._pmid_servo.set_angle(0)
#           self._smid_servo.set_angle(0)
            self._paft_servo.set_angle(self._rotate_angle)
            self._saft_servo.set_angle(-1 * self._rotate_angle)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_omnidirectional_angle(self, angle):
        '''
        Sets all servos to the same value.
        '''
        _angle = angle #int(angle * Servo.SCALE_FACTOR)
        self._log.info(Fore.BLACK + 'set servo positions to {:.2f}° (called with {:.2f}°)'.format(_angle, angle) + Style.RESET_ALL)
        if USE_THREADING:
            _t_pfwd = Thread(target = self._pfwd_servo.set_angle, args=[_angle], name='set_pfwd', daemon=self._is_daemon)
            _t_sfwd = Thread(target = self._sfwd_servo.set_angle, args=[_angle], name='set_sfwd', daemon=self._is_daemon)
#           _t_pmid = Thread(target = self._pmid_servo.set_angle, args=[_angle], name='set_pmid', daemon=self._is_daemon)
#           _t_smid = Thread(target = self._smid_servo.set_angle, args=[_angle], name='set_smid', daemon=self._is_daemon)
            _t_paft = Thread(target = self._paft_servo.set_angle, args=[_angle], name='set_paft', daemon=self._is_daemon)
            _t_saft = Thread(target = self._saft_servo.set_angle, args=[_angle], name='set_saft', daemon=self._is_daemon)
            _t_pfwd.start()
            _t_sfwd.start()
#           _t_pmid.start()
#           _t_smid.start()
            _t_paft.start()
            _t_saft.start()
        else:
            self._pfwd_servo.set_angle(_angle)
            self._sfwd_servo.set_angle(_angle)
#           self._pmid_servo.set_angle(_angle)
#           self._smid_servo.set_angle(_angle)
            self._paft_servo.set_angle(_angle)
            self._saft_servo.set_angle(_angle)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def calculate_steering(self, inside_steering_angle):
        '''
        author: Jon Hylands
        source: https://discord.com/channels/835614845129719828/835618978096218112/1243244829866197042
        '''
        if inside_steering_angle == 0:
            return 0

        # First, inside radius
        inside_steering_rads = math.radians(inside_steering_angle)
        inside_distance = self._half_length / (math.tan(inside_steering_rads))
        inside_hyp = math.sqrt(self._half_length ** 2 + inside_distance ** 2)
        inside_radius = inside_hyp - self._wheel_offset

        # Second, outside radius
        outside_distance = inside_distance + (self._half_width * 2)
        outside_hyp = math.sqrt(outside_distance ** 2 + self._half_length ** 2)
        outside_radius = outside_hyp + self._wheel_offset

        # Finally, outside angle
        outside_steering_rads = math.atan2(self._half_length, outside_distance)
        outside_steering_angle = math.degrees(outside_steering_rads)

#       return (inside_radius, outside_steering_angle, outside_radius)
        return outside_steering_angle

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_afrs_angle(self, angle):
        '''
        Sets all servos in AFRS steering mode.

        Positive values turn towards port (counter-clockwise), starboard is the outer side.

        Negative values turn towards starboard (clockwise), port is the outer side.

        When we are turning the inner wheels have a tighter turning radius than the outer wheels.
        '''
        _angle = angle
        _opposite_angle = int(self.calculate_steering(abs(_angle)))
        self._log.info('set servo positions to ' + Fore.WHITE + Style.BRIGHT + '{}°; opposite: {}°'.format(_angle, _opposite_angle) + Style.RESET_ALL)
        if _angle < 0: # if negative, stbd is inner, port is outer
            _port_angle = _angle
            _stbd_angle = _opposite_angle
        else: # if positive, port is inner, stbd is outer (zero is same)
            _port_angle = _opposite_angle
            _stbd_angle = -1 * _angle
        if USE_THREADING:
            _t_pfwd = Thread(target = self._pfwd_servo.set_angle, args=[-1 * _port_angle], name='set_pfwd', daemon=self._is_daemon)
            _t_sfwd = Thread(target = self._sfwd_servo.set_angle, args=[_stbd_angle], name='set_sfwd', daemon=self._is_daemon)
#           _t_pmid = Thread(target = self._pmid_servo.set_angle, args=[0], name='set_pmid', daemon=self._is_daemon)
#           _t_smid = Thread(target = self._smid_servo.set_angle, args=[0], name='set_smid', daemon=self._is_daemon)
            _t_paft = Thread(target = self._paft_servo.set_angle, args=[_port_angle], name='set_paft', daemon=self._is_daemon)
            _t_saft = Thread(target = self._saft_servo.set_angle, args=[-1 * _stbd_angle], name='set_saft', daemon=self._is_daemon)
            _t_pfwd.start()
            _t_sfwd.start()
#           _t_pmid.start()
#           _t_smid.start()
            _t_paft.start()
            _t_saft.start()
        else:
            self._pfwd_servo.set_angle(-1 * _port_angle)
            self._sfwd_servo.set_angle(_stbd_angle)
#           self._pmid_servo.set_angle(0)
#           self._smid_servo.set_angle(0)
            self._paft_servo.set_angle(_port_angle)
            self._saft_servo.set_angle(-1 * _stbd_angle)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_angle(self, orientation, angle):
        '''
        Sets the servo angle for a single servo with the given orientation.
        '''
        if not isinstance(angle, int): # TEMP
            raise Exception('angle argument is not an integer.')
        if not self.enabled:
            self._log.error('servo controller not enabled.')
            return
        elif orientation == Orientation.SFWD:
            self._sfwd_servo.set_angle(int(angle))
        elif orientation == Orientation.PFWD:
            self._pfwd_servo.set_angle(int(angle))
#       elif orientation == Orientation.SMID:
#           self._smid_servo.set_angle(int(angle))
#       elif orientation == Orientation.PMID:
#           self._pmid_servo.set_angle(int(angle))
        elif orientation == Orientation.SAFT:
            self._saft_servo.set_angle(int(angle))
        elif orientation == Orientation.PAFT:
            self._paft_servo.set_angle(int(angle))
        else:
            raise TypeError('unsupported orientation: {}'.format(orientation))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enables the motors. This issues a warning if already enabled, but no
        harm is done in calling it repeatedly.
        '''
        if self.enabled:
            self._log.warning('already enabled.')
        else:
            Component.enable(self)
            self._log.info('enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable the motors, halting first if in motion.
        '''
        if self.enabled:
            self._log.info('disabling…')
#           self.stop_loop() # stop loop thread
            Component.disable(self)
            self._log.info('disabled.')
        else:
            self._log.debug('already disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _recenter(self, servo):
        '''
        Slowly recenters the servo using a Thread.
        '''
        _angle = int(servo.angle)
        if _angle > 0:
            self._log.info('recentering servo {}… (-)'.format(servo.name))
            for a in range(_angle, 0, -1):
                servo.set_angle(a)
                time.sleep(ServoController.ROTATION_DELAY_MS / 1000)
        elif _angle < 0:
            self._log.info('recentering servo {}… (+)'.format(servo.name))
            for a in range(_angle, 0):
                servo.set_angle(a)
                time.sleep(ServoController.ROTATION_DELAY_MS / 1000)
        self._log.info(Fore.CYAN + 'servo {} recentering complete.'.format(servo.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def recenter(self):
        ''' Slowly recenters the servos.
        '''
        # TODO if USE_THREADING:
        if not self.closed:
            _t_pfwd = Thread(target = self._recenter, args=[self._pfwd_servo], name='recenter_pfwd', daemon=False)
            _t_sfwd = Thread(target = self._recenter, args=[self._sfwd_servo], name='recenter_sfwd', daemon=False)
#           _t_pmid = Thread(target = self._recenter, args=[self._pmid_servo], name='recenter_pmid', daemon=False)
#           _t_smid = Thread(target = self._recenter, args=[self._smid_servo], name='recenter_smid', daemon=False)
            _t_paft = Thread(target = self._recenter, args=[self._paft_servo], name='recenter_paft', daemon=False)
            _t_saft = Thread(target = self._recenter, args=[self._saft_servo], name='recenter_saft', daemon=False)
            _t_pfwd.start()
            _t_sfwd.start()
#           _t_pmid.start()
#           _t_smid.start()
            _t_paft.start()
            _t_saft.start()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Quickly centers the servos and close.
        '''
        if not self.closed:
            return
            self._pfwd_servo.center()
            self._sfwd_servo.center()
#           self._pmid_servo.center()
#           self._smid_servo.center()
            self._paft_servo.center()
            self._saft_servo.center()
            Component.close(self) # calls disable

#EOF
