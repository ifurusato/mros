#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-08-22
# modified: 2021-08-22
#

import time
import itertools
from datetime import datetime as dt
from threading import Thread
from math import isclose
from colorama import init, Fore, Style
init()

from core.chadburn import Chadburn
from core.orientation import Orientation
from core.logger import Level, Logger
from core.rate import Rate
from core.rotation import Rotation
from core.steering_mode import SteeringMode
from hardware.sound import Sound
from hardware.player import Player

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Calibrator(object):
    '''
    A ballistic behaviour that rotates the robot horizontally in place,
    the requirement for calibrating the ICM20948 IMU.

    This populates a stack of callback functions to be executed in sequence:

      1. set the steering to rotation mode
      2. rotate the robot clockwise roughly 360 degrees
      3. set the steering back to AFRS mode
      4. flood the IMU queue with new values
      5. check and set the calibration status of the IMU

    :param motion_controller:     The MotionController backing the calibration.
    :param level:      The log level.
    '''
    def __init__(self, config, motion_controller, level=Level.INFO):
        super().__init__()
        if not isinstance(level, Level):
            raise ValueError('wrong type for level argument: {}'.format(type(level)))
        self._log = Logger('calibrator', level)
        self._motion_controller = motion_controller
        self._play_sound = config['mros'].get('play_sound')
        _cfg = config.get('mros').get('calibrator')
        self._step_limit = _cfg.get('step_limit') # roughly one rotation
        self._break_time_ms = _cfg.get('break_time_ms')
        if self._motion_controller.imu is None:
            raise Exception('cannot calibrate: IMU has not been set.')
        self._icm20948 = self._motion_controller.imu.icm20948
        if not self._icm20948.enabled:
            self._icm20948.enable()
            self._log.info('enabled ICM20948 IMU.')
        self._flood_queue_thread = None
        self._wait_start = None
        # create a stack of callbacks
        self._stack = []
        self._stack.append(self._cleanup)
        self._stack.append(self._check_calibration)
        self._stack.append(self._stabilise_queue)
        self._stack.append(self._reposition_afrs)
        self._stack.append(self._rotate_in_place)
        self._stack.append(self._reposition_rotate)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def check_calibration(self):
        '''
        This clears any existing content in the queue, then floods the queue
        with new sensor values to determine if the IMU is currently calibrated.

        This starts a thread that calls this method again as its callback.
        '''
        self._log.info('checking calibration…')
        if self._flood_queue_thread:
            # then this is the callback
            self._log.info('received callback from thread.')
            self._motion_controller.set_calibrated()
            self._flood_queue_thread = None
        else:
            # initial, call thread with this method as callback
            self._icm20948.clear_queue()
            self._log.info('checking calibration…')
            self._flood_queue_thread = Thread(name='flood_queue_thread', target=Calibrator._flood_queue, args=[self, self.check_calibration], daemon=True)
            self._flood_queue_thread.start()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _pop_calibrate_stack(self):
        if len(self._stack) > 0:
            _method = self._stack.pop()
            self._log.info(Style.DIM + 'executing {} method from stack…'.format(_method.__name__))
            _method()
        else:
            # in theory we should be re-enabling bumpers after completion
            self._log.info('calibrator complete.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def calibrate(self):
        self._log.info('calibrating…')
        if self._play_sound:
            Player.instance().play_from_thread(Sound.TELEMETRY)
        # in theory we should be disabling bumpers prior to calibration
        self._pop_calibrate_stack()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _reposition_rotate(self):
        self._log.info('🍄 1. reposition for rotation…')
        # first step is reposition for rotation...
        self._motion_controller.reposition(SteeringMode.ROTATE, self._pop_calibrate_stack)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _rotate_in_place(self):
        self._log.info('🍄 2. calling rotate in place movement…')
        self._rotate_thread = Thread(name='rotate_thread', target=Calibrator._rotation_movement, args=[self, self._pop_calibrate_stack], daemon=True)
        self._rotate_thread.start()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _rotation_movement(self, callback):
        '''
        Performs the physical movement of rotating the robot clockwise through
        a complete circle, mimicking the accumulation of statistics to determine
        following the rotation if the values have approached stability.
        '''
        self._log.info('begin calibration movement loop…')

        self._motion_controller.rotate(Rotation.CLOCKWISE)

        _max_delta = 380.0
        _diff = 0.0
        _delta = 0.0
        _initial_heading = self._icm20948.uncalibrated_heading

        # choose a motor
        _motor = self._motion_controller.motor_controller.get_motor(Orientation.SAFT)
        _step_count = 0
        _steps = _motor.steps
        self._log.info(Fore.GREEN + 'INITIAL heading value: {:4.2f}°; {} steps.'.format(_initial_heading, _steps))

        _startup = 50 # after which, full speed
        _min_speed_clamp = lambda n: max(min(Chadburn.DEAD_SLOW_AHEAD.speed, n), Chadburn.DEAD_SLOW_ASTERN.speed)
        _max_speed_clamp = lambda n: max(min(Chadburn.HALF_AHEAD.speed, n), Chadburn.HALF_ASTERN.speed)

        # loop to rotate in place...
        _counter = itertools.count()
        _hz = 20
        _rate = Rate(_hz, Level.ERROR)

        while _step_count < self._step_limit:
            _count = next(_counter)
            # when rotating clockwise, the heading value should only increase...
            _steps = _motor.steps
            _step_count += abs(_steps)
            _heading = self._icm20948.uncalibrated_heading
            # add to queue
            if self._icm20948.calibration_check(_heading):
                self._log.info(Fore.GREEN + Style.BRIGHT + 'IMU was calibrated while moving.')
                _step_count = self._step_limit + 1
                self._motion_controller.motor_controller.set_speed(Orientation.PORT, 0.0)
                self._motion_controller.motor_controller.set_speed(Orientation.STBD, 0.0)
            # now set target speed of motors while rotating...
            _target_speed = Chadburn.HALF_AHEAD.speed # self._motion_controller.get_target_speed()
            if _step_count < _startup:
                _target_speed *= 0.5
            if isclose(_target_speed, 0.0, abs_tol=1e-4):
                self._motion_controller.motor_controller.set_speed(Orientation.PORT, 0.0)
                self._motion_controller.motor_controller.set_speed(Orientation.STBD, 0.0)
            else:
                self._motion_controller.motor_controller.set_speed(Orientation.PORT, _target_speed)
                self._motion_controller.motor_controller.set_speed(Orientation.STBD, _target_speed)
#               self._log.info(Fore.CYAN + 'target speed: {:4.2f}; '.format(_target_speed))
            if self._icm20948.is_calibrated:
                _cal_msg = Fore.GREEN + 'calibrated.'
            else:
                _cal_msg = Style.DIM + 'not calibrated.'
            self._log.info('[{:d}] heading: {:4.2f}°; {} steps; count: {}/{} steps; {}'.format(_count, _heading, _steps, _step_count, self._step_limit, _cal_msg))
            _rate.wait()
            # end loop ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        self._motion_controller.motor_controller.set_speed(Orientation.PORT, 0.0)
        self._motion_controller.motor_controller.set_speed(Orientation.STBD, 0.0)
        self._log.info('exited loop after {} steps.'.format(_step_count))
        if not self._motion_controller.motor_controller.all_motors_are_stopped:
            self._motion_controller.motor_controller.stop()
        self._log.info('calibrate loop end.')
        if callback:
            callback()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _reposition_afrs(self):
        self._log.info('🍄 3. reposition for afrs…')
        self._motion_controller.reposition(SteeringMode.AFRS, self._pop_calibrate_stack)
        self._log.info('recentered.')
        # make sure we're back to normal steering
        self._motion_controller.rotate(Rotation.STOPPED)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _stabilise_queue(self):
        self._log.info('🍄 4. stabilise…')
        self._flood_queue_thread = Thread(name='flood_queue_thread', target=Calibrator._flood_queue, args=[self, self._pop_calibrate_stack], daemon=True)
        self._flood_queue_thread.start()

    def _flood_queue(self, callback):
        '''
        Replaces all the heading values in the queue.
        '''
        self._wait_start = dt.now()
        self._log.info('flooding queue…')
        for i in range(self._icm20948.queue_length + 10):
            _heading = self._icm20948.uncalibrated_heading
#           self._log.info('flooding heading: {:4.2f}'.format(_heading))
            _calibrated = self._icm20948.calibration_check(_heading) 
            # can break early but it's not necessary
            if _calibrated:
                self._log.info('breaking flood queue loop early.')
                break
            time.sleep(0.01)
        if callback:
            callback()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _check_calibration(self):
        '''
        Completes the prescribed waiting time before checking/setting the
        calibration state.
        '''
        self._log.info('checking calibration state…')
        while (dt.now() - self._wait_start).total_seconds() * 1000 < self._break_time_ms:
            self._log.info(Style.DIM + '{:d}ms elapsed…'.format(int((dt.now() - self._wait_start).total_seconds() * 1000)))
            if self._icm20948.is_calibrated:
                self._log.info(Fore.MAGENTA + 'calibrated: breaking wait loop…')
                break
            time.sleep(0.1)
        self._motion_controller.set_calibrated()
        self._log.info('calibration status set.')
        self._pop_calibrate_stack() # call cleanup

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _cleanup(self):
        self._log.info('cleanup…')
        self._motion_controller.motor_controller.clear_speed_multipliers()

#EOF
