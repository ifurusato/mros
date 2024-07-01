#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-22
# modified: 2024-06-03
#

import sys, time, traceback
from threading import Thread
from enum import Enum
import asyncio, random
from datetime import datetime as dt

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
from hardware.slew_rate import SlewRate


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class ServoController(Component):
    USE_THREADING          = True
    USE_THREADING_FOR_AFRS = False
    RECENTER_USE_LOOP      = False
    RECENTER_USE_THREADING = True
    SLOW_RECENTER_ON_CLOSE = True
    ROTATION_DELAY_MS      = 50
    '''
    A servo controller that asbtracts the actual control of servos from
    the physical servo control hardware.

    This relies upon both a SlewLimiter and JerkLimiter so that velocity
    and power (resp.) changes occur gradually and safely.

    :param config:          the application configuration
    :param level:           the logging level
    '''

    def __init__(self, config, level=Level.INFO):
        self._log = Logger('servo-ctrl', level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._log.info('initialising servos…')
        if not isinstance(config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(name)))
        self._is_daemon = True
        _cfg = config.get('mros').get('servo_controller')
        self._rotate_angle = _cfg.get('rotate_angle') # angle for rotate mode
        self._spin_angle   = _cfg.get('spin_angle') # angle for rotate mode
        self._pfwd_servo = Servo(_cfg, Orientation.PFWD, True, level)
        self._sfwd_servo = Servo(_cfg, Orientation.SFWD, True, level)
        self._pmid_servo = None # Servo(_cfg, Orientation.PMID, True, level)
        self._smid_servo = None # Servo(_cfg, Orientation.SMID, True, level)
        self._paft_servo = Servo(_cfg, Orientation.PAFT, True, level)
        self._saft_servo = Servo(_cfg, Orientation.SAFT, True, level)
        self._all_servos = self._get_servos()
        # default mode
        self._steering_mode = None
        self.set_mode(SteeringMode.SKID)
        self._log.info('servo controller ready with {} servos.'.format(len(self._all_servos)))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return 'servo-ctrl'

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def report(self):
        _servos = self.get_servos()
        self._log.info(Fore.MAGENTA + 'report on {} servos:'.format(len(_servos)))
        for _servo in _servos:
            _servo.report()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def steering_mode(self):
        return self._steering_mode

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
        if steering_mode is None:
            raise ValueError('null steering mode argument.')
        elif steering_mode is SteeringMode.ACKERMANN:
            raise Exception('unsupported steering mode: {}'.format(steering_mode.name))
        elif steering_mode is SteeringMode.SKID:
            self._steering_mode = SteeringMode.SKID
            self.recenter(False)
        elif steering_mode is SteeringMode.AFRS:
            self._steering_mode = SteeringMode.AFRS
            pass
        elif steering_mode is SteeringMode.OMNIDIRECTIONAL:
            raise Exception('unsupported steering mode: {}'.format(steering_mode.name))
        elif steering_mode is SteeringMode.SIDEWAYS:
            raise Exception('unsupported steering mode: {}'.format(steering_mode.name))
        elif steering_mode is SteeringMode.ROTATE:
            self._steering_mode = SteeringMode.ROTATE
            self.set_rotate_mode()
        elif steering_mode is SteeringMode.FORWARD_PIVOT:
            raise Exception('unsupported steering mode: {}'.format(steering_mode.name))
        elif steering_mode is SteeringMode.AFT_PIVOT:
            raise Exception('unsupported steering mode: {}'.format(steering_mode.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_ackermann_angle(self, angle):
        raise Exception('unsupported steering mode: ackermann')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _set_rotate(self, servo, angle):
        if not isinstance(angle, int): # TEMP
            angle = int(angle)
        _start_time = dt.now()
        _current_angle = int(servo.angle)
        self._log.info(Fore.MAGENTA + 'set_rotate: servo {} angle {:d}°; to: {:d}°'.format(servo.name, _current_angle, angle))
        if _current_angle == angle:
            # no change
            pass
        elif _current_angle > angle:
#           self._log.info('set rotate for servo {} from {:d}° DOWN to {:d}°'.format(servo.name, _current_angle, angle))
            for a in range(_current_angle, angle, -1):
                servo.angle = a
                time.sleep(ServoController.ROTATION_DELAY_MS / 1000)
        elif _current_angle < angle:
#           self._log.info('set rotate for servo {} from {:d}° UP to {:d}°'.format(servo.name, _current_angle, angle))
            for a in range(_current_angle, angle):
                servo.angle = a
                time.sleep(ServoController.ROTATION_DELAY_MS / 1000)
            self._log.info('servo {} rotate complete.'.format(servo.name))
        self._log.info(Fore.MAGENTA + 'AFTER: set_rotate: servo {} angle {:d}°; from: {:d}°'.format(servo.name, int(servo.angle), angle))
        _elapsed_ms = round(( dt.now() - _start_time ).total_seconds() * 1000.0)
        self._log.info(Fore.YELLOW + 'set rotate complete: elapsed: {:d}ms'.format(_elapsed_ms))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_rotate_mode_for_servo(self, orientation):
        self._log.info(Fore.YELLOW + 'set rotate for servo: {}'.format(orientation))
        if orientation is Orientation.PFWD or orientation is Orientation.SAFT: 
            _multiplier = -1
        else:
            _multiplier = 1
        _servo = self.get_servo(orientation)
        _name = 'rotate_' + _servo.name
        _thread = Thread(target = self._set_rotate, args=[_servo, _multiplier * self._rotate_angle], name=_name, daemon=self._is_daemon)
        _thread.start()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_rotate_mode(self):
        if ServoController.USE_THREADING:
            _t_pfwd = Thread(target = self._set_rotate, args=[self._pfwd_servo, -1 * self._rotate_angle], name='rotate_pfwd', daemon=self._is_daemon)
            _t_sfwd = Thread(target = self._set_rotate, args=[self._sfwd_servo, self._rotate_angle], name='rotate_sfwd', daemon=self._is_daemon)
#           _t_pmid = Thread(target = self._set_rotate, args=[self._pmid_servo, 0], name='rotate_pmid', daemon=self._is_daemon)
#           _t_smid = Thread(target = self._set_rotate, args=[self._smid_servo, 0], name='rotate_smid', daemon=self._is_daemon)
            _t_paft = Thread(target = self._set_rotate, args=[self._paft_servo, self._rotate_angle], name='rotate_paft', daemon=self._is_daemon)
            _t_saft = Thread(target = self._set_rotate, args=[self._saft_servo, -1 * self._rotate_angle], name='rotate_saft', daemon=self._is_daemon)
            _t_pfwd.start()
            _t_sfwd.start()
#           _t_pmid.start()
#           _t_smid.start()
            _t_paft.start()
            _t_saft.start()
        else:
            self._pfwd_servo.angle = -1 * self._rotate_angle
            self._sfwd_servo.angle = self._rotate_angle
#           self._pmid_servo.angle = 0
#           self._smid_servo.angle = 0
            self._paft_servo.angle = self._rotate_angle
            self._saft_servo.angle = -1 * self._rotate_angle

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_omnidirectional_angle(self, angle):
        '''
        Sets all servos to the same value.
        '''
        _angle = angle #int(angle * Servo.SCALE_FACTOR)
        self._log.info(Fore.BLACK + 'set servo positions to {:.2f}° (called with {:.2f}°)'.format(_angle, angle))
        if ServoController.USE_THREADING:
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
            self._pfwd_servo.angle = _angle
            self._sfwd_servo.angle = _angle
#           self._pmid_servo.angle = _angle
#           self._smid_servo.angle = _angle
            self._paft_servo.angle = _angle
            self._saft_servo.angle = _angle

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_afrs_angle(self, port_angle, stbd_angle):
        '''
        Sets all servos in AFRS steering mode.

        Positive values turn towards port (counter-clockwise), starboard is the outer side.
        Negative values turn towards starboard (clockwise), port is the outer side.
        '''
        self._log.info('set afrs servo to: ' + Fore.RED + 'port: {}°; '.format(port_angle) + Fore.GREEN + 'stbd {}°'.format(stbd_angle))
#       if self._afrs_angle < 0: # if negative, stbd is inner, port is outer
#           port_angle = self._afrs_angle
#           stbd_angle = self._opposite_afrs_angle
#       else: # if positive, port is inner, stbd is outer (zero is same)
#           port_angle = self._opposite_afrs_angle
#           stbd_angle = -1 * self._afrs_angle
        if port_angle < 0:
            stbd_angle = -1 * stbd_angle

        if ServoController.USE_THREADING_FOR_AFRS:
            _t_pfwd = Thread(target = self._pfwd_servo.set_angle, args=[-1 * port_angle], name='set_pfwd', daemon=self._is_daemon)
            _t_sfwd = Thread(target = self._sfwd_servo.set_angle, args=[stbd_angle], name='set_sfwd', daemon=self._is_daemon)
#           _t_pmid = Thread(target = self._pmid_servo.set_angle, args=[0], name='set_pmid', daemon=self._is_daemon)
#           _t_smid = Thread(target = self._smid_servo.set_angle, args=[0], name='set_smid', daemon=self._is_daemon)
            _t_paft = Thread(target = self._paft_servo.set_angle, args=[port_angle], name='set_paft', daemon=self._is_daemon)
            _t_saft = Thread(target = self._saft_servo.set_angle, args=[-1 * stbd_angle], name='set_saft', daemon=self._is_daemon)
            _t_pfwd.start()
            _t_sfwd.start()
#           _t_pmid.start()
#           _t_smid.start()
            _t_paft.start()
            _t_saft.start()
        else:
            self._pfwd_servo.angle = -1 * port_angle
            self._sfwd_servo.angle = -1 * stbd_angle
#           self._pmid_servo.angle = 0
#           self._smid_servo.angle = 0
            self._paft_servo.angle = port_angle
            self._saft_servo.angle = stbd_angle

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
            self._sfwd_servo.angle = int(angle)
        elif orientation == Orientation.PFWD:
            self._pfwd_servo.angle = int(angle)
#       elif orientation == Orientation.SMID:
#           self._smid_servo.angle = int(angle)
#       elif orientation == Orientation.PMID:
#           self._pmid_servo.angle = int(angle)
        elif orientation == Orientation.SAFT:
            self._saft_servo.angle = int(angle)
        elif orientation == Orientation.PAFT:
            self._paft_servo.angle = int(angle)
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
            for _servo in self._all_servos:
                if _servo:
                    _servo.enable()
            self.recenter(False)
            self._log.info('enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _recenter(self, servo, close_upon_completion):
        '''
        Slowly recenters the servo. This is suitable for being called by a Thread.
        '''
        try:
#           _angle = int(servo.angle)
            _angle = int(servo.adjusted_angle)
            _adj_angle = int(servo.adjusted_angle)
            self._log.info(Fore.MAGENTA + 'starting angle of servo {}: {:d}°; adj: {:d}°'.format(servo.name, _angle, _adj_angle))
            if _angle == 0:
                self._log.info('servo {} is centered.'.format(servo.name))
            elif _angle > 0:
                self._log.info('recentering servo {} from {:d} to {:d}… (-) [current: {:d}]'.format(servo.name, _angle, 0, servo.adjusted_angle))
                for a in range(_angle, 0, -1):
                    servo.angle = a
                    time.sleep(ServoController.ROTATION_DELAY_MS / 1000)
                self._log.info('recentered {} servo.'.format(servo.name))
            elif _angle < 0:
                self._log.info('recentering servo {} from {:d} to {:d}… (+) [current: {:d}]'.format(servo.name, _angle, 0, servo.adjusted_angle))
                for a in range(_angle, 0):
                    servo.angle = a
                    time.sleep(ServoController.ROTATION_DELAY_MS / 1000)
                self._log.info('recentered {} servo.'.format(servo.name))
            if close_upon_completion:
                servo.disable()
        except Exception as e:
            self._log.error('{} encountered while recentering: {}\n{}'.format(type(e), e, traceback.format_exc())) 
        finally:
            servo.angle = 0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _recenter_on_close(self):
        '''
        Recenters all servos, blocking until they're finished.
        '''
        _count = 0
        self.recenter(True)
        while _count < 30 and not self.all_disabled():
            _count += 1
            self._log.info(Style.DIM + '[{:02d}] recentering servos…'.format(_count))
            time.sleep(0.2)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def recenter(self, close_upon_completion):
        '''
        Slowly recenters the servos. if 'is_daemon' is True, will block
        application exit until completed.
        '''
        if self.closed:
            return
        if ServoController.RECENTER_USE_THREADING:
            # if close upon completion we want non-daemonic threads so they complete
            _is_daemon = not close_upon_completion
            _t_pfwd = Thread(target = self._recenter, args=[self._pfwd_servo, close_upon_completion], name='recenter_pfwd', daemon=_is_daemon)
            _t_sfwd = Thread(target = self._recenter, args=[self._sfwd_servo, close_upon_completion], name='recenter_sfwd', daemon=_is_daemon)
#           _t_pmid = Thread(target = self._recenter, args=[self._pmid_servo, close_upon_completion], name='recenter_pmid', daemon=_is_daemon)
#           _t_smid = Thread(target = self._recenter, args=[self._smid_servo, close_upon_completion], name='recenter_smid', daemon=_is_daemon)
            _t_paft = Thread(target = self._recenter, args=[self._paft_servo, close_upon_completion], name='recenter_paft', daemon=_is_daemon)
            _t_saft = Thread(target = self._recenter, args=[self._saft_servo, close_upon_completion], name='recenter_saft', daemon=_is_daemon)
            _t_pfwd.start()
            _t_sfwd.start()
#           _t_pmid.start()
#           _t_smid.start()
            _t_paft.start()
            _t_saft.start()
        elif ServoController.RECENTER_USE_LOOP:
            self._recenter(self._pfwd_servo)
            self._recenter(self._sfwd_servo)
#           self._recenter(self._pmid_servo)
#           self._recenter(self._smid_servo)
            self._recenter(self._paft_servo)
            self._recenter(self._saft_servo)
        else:
            self._pfwd_servo.angle = 0
            self._sfwd_servo.angle = 0
#           self._pmid_servo.angle = 0
#           self._smid_servo.angle = 0
            self._paft_servo.angle = 0
            self._saft_servo.angle = 0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def all_disabled(self):
        for _servo in self._all_servos:
            if _servo:
                if _servo.enabled:
                    return False
        return True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable the servos, recentering if not at zero.
        '''
        if self.enabled:
            self._log.info('disabling…')
            if ServoController.SLOW_RECENTER_ON_CLOSE:
                self._recenter_on_close()
            else:
                if self._pfwd_servo.angle != 0:
                    self._pfwd_servo.center()
                if self._sfwd_servo.angle != 0:
                    self._sfwd_servo.center()
#               if self._pmid_servo.angle != 0:
#                   self._pmid_servo.center()
#               if self._smid_servo.angle != 0:
#                   self._smid_servo.center()
                if self._paft_servo.angle != 0:
                    self._paft_servo.center()
                if self._saft_servo.angle != 0:
                    self._saft_servo.center()
            for _servo in self._all_servos:
                if _servo:
                    _servo.disable()
            Component.disable(self)
            self._log.info('disabled.')
        else:
            self._log.debug('already disabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Quickly centers the servos and close.
        '''
        if not self.closed:
            Component.close(self) # calls disable

#EOF
