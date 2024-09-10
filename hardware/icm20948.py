#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-20
# modified: 2024-05-27
#

import traceback
import itertools
import math, statistics
from math import pi as π
from collections import deque
from datetime import datetime as dt
from colorsys import hsv_to_rgb
from colorama import init, Fore, Style
init()

from icm20948 import ICM20948
from rgbmatrix5x5 import RGBMatrix5x5
from matrix11x7 import Matrix11x7
from matrix11x7.fonts import font3x5, font5x5, font5x7, font5x7smoothed

from core.cardinal import Cardinal
from core.convert import Convert
from core.component import Component
from core.logger import Logger, Level
from core.orientation import Orientation
from core.rate import Rate
from core.ranger import Ranger
from hardware.rgbmatrix import RgbMatrix, DisplayType
from hardware.sound import Sound
from hardware.player import Player

IN_MIN  = 0.0      # minimum analog value from IO Expander
IN_MAX  = 3.3      # maximum analog value from IO Expander
OUT_MIN = -1.0 * π # minimum scaled output value
OUT_MAX = π        # maximum scaled output value
HALF_PI = π / 2.0

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Icm20948(Component):
    '''
    Wraps the functionality of an ICM20948 IMU largely as a compass, though
    pitch and roll are also available. This includes optional trim adjustment,
    a calibration check, an optional console, numeric and color displays of
    heading, as well as making raw accelerometer and gyroscope values available.

    :param config:          the application configuration
    :param rgbmatrix        the optional RgbMatrix to indicate calibration
    :param level            the log level
    '''
    def __init__(self, config, rgbmatrix=None, level=Level.INFO):
        self._log = Logger('icm20948', level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._log.info('initialising icm20948…')
        if not isinstance(config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(name)))
        # add color display ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._rgbmatrix          = rgbmatrix
        if self._rgbmatrix:
            if not isinstance(self._rgbmatrix, RgbMatrix):
                raise ValueError('wrong type for RgbMatrix argument: {}'.format(type(self._rgbmatrix)))
            self._rgbmatrix5x5 = self._rgbmatrix.get_rgbmatrix(Orientation.PORT)
        else:
            self._rgbmatrix5x5 = None
        self._counter = itertools.count()
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _cfg = config['mros'].get('hardware').get('icm20948')
        self._verbose            = _cfg.get('verbose')
        self._adjust_trim        = _cfg.get('adjust_trim')
        self._show_console       = _cfg.get('show_console')
        self._show_rgbmatrix5x5  = _cfg.get('show_rgbmatrix5x5')
        self._show_rgbmatrix11x7 = _cfg.get('show_rgbmatrix11x7')
        self._play_sound         = _cfg.get('play_sound') # if True, play sound to indicate calibration
        if self._play_sound:
            self._player = Player.instance()
        else:
            self._player = None
        # set up trim control ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._pitch_trim = _cfg.get('pitch_trim') # 0.0
        self._roll_trim  = _cfg.get('roll_trim') # 4.0
        self._trim = 0.0
        if not self._adjust_trim:
            # use fixed value
            self._trim = _cfg.get('heading_trim')
        self._digital_pot = None
        if self._adjust_trim:
            # configure potentiometer ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            self._digital_pot = self._component_registry.get('digital-pot-0x0E')
            if self._digital_pot:
                self._digital_pot.set_output_range(OUT_MIN, OUT_MAX)
        # add numeric display ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._low_brightness    = 0.15
        self._medium_brightness = 0.25
        self._high_brightness   = 0.45
        if self._show_rgbmatrix11x7:
            self._matrix11x7 = Matrix11x7()
            self._matrix11x7.set_brightness(self._low_brightness)
        self._cardinal_tolerance = _cfg.get('cardinal_tolerance') # tolerance to cardinal points (in radians)
        self._log.info('cardinal tolerance: {:.8f}'.format(self._cardinal_tolerance))
        # general orientation ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._X = 0
        self._Y = 1
        self._Z = 2
        # The two axes which relate to heading depend on orientation of the
        # sensor, think Left & Right, Forwards and Back, ignoring Up and Down.
        # When the sensor is sitting vertically upright in a Breakout Garden
        # socket, use (Z,Y), where hanging upside down would be (Y,Z).
        self._axes = self._Z, self._Y
        # queue for stability check stats ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._stdev = 0.0
        self._queue_length = _cfg.get('queue_length') # also affects how fast mean catches up to data
#       self._queue = deque(self._queue_length*[0], self._queue_length)
        self._queue = deque([], self._queue_length)
        self._stability_threshold = _cfg.get('stability_threshold')
        # misc/variables ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._heading_count = 0
        self._display_rate = 20 # display every 10th set of values
        self._poll_rate_hz = _cfg.get('poll_rate_hz')
        self._radians = None
        self._amin = None
        self._amax = None
        self._pitch = 0.0
        self._roll  = 0.0
        self._heading = 0
        self._formatted_heading = lambda: 'Heading: {:d}°'.format(self._heading)
        self._mean_heading = 0
        self._mean_heading_radians = None
        self._accel = [0.0, 0.0, 0.0]
        self._gyro =  [0.0, 0.0, 0.0]
        self._include_accel_gyro = _cfg.get('include_accel_gyro')
        self._is_calibrated = False
        # instantiate sensor class  ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self.__icm20948 = ICM20948(i2c_addr=_cfg.get('i2c_address'))
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def queue_length(self):
        return self._queue_length

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def is_calibrated(self):
        '''
        Return true if the compass is calibrated. This generally requires
        turning the sensor through 360° to set minimum and maximum values,
        and then waiting unmoving for it to settle, until the standard
        deviation of a queue of values falls below a configured threshold.
        '''
        return self._is_calibrated

    def set_is_calibrated(self, calibrated):
        '''
        Externally set the value of the state of calibration of the ICM20948.
        This is done by processes that utilise the ICM20948 but themselves
        determine the calibration state.
        '''
        self._is_calibrated = calibrated

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def is_cardinal_aligned(self, cardinal=None):
        '''
        Returns True if the mean heading is aligned within a 3° tolerance to
        the specified cardinal directory, or if the argument is None, any of
        the four cardinal directions.
        '''
        if self._mean_heading_radians is None:
            return False
        _angle = self._mean_heading_radians
        _degrees = int(Convert.to_degrees(_angle))
        if cardinal is None:
            print('angle: {:4.2f}r / {:d}°; cardinal: not provided'.format(_angle, _degrees))
        else:
            print('angle: {:4.2f}r / {:d}°; cardinal: {:4.2f}'.format(_angle, _degrees, cardinal.radians))
        if (( cardinal is None or cardinal is Cardinal.NORTH )
                    and math.isclose(_angle, Cardinal.NORTH.radians, abs_tol=self._cardinal_tolerance)):
            return True
        elif (( cardinal is None or cardinal is Cardinal.WEST )
                    and math.isclose(_angle, Cardinal.WEST.radians, abs_tol=self._cardinal_tolerance)):
            return True
        elif (( cardinal is None or cardinal is Cardinal.SOUTH )
                    and math.isclose(_angle, Cardinal.SOUTH.radians, abs_tol=self._cardinal_tolerance)):
            return True
        elif (( cardinal is None or cardinal is Cardinal.EAST )
                    and math.isclose(_angle, Cardinal.EAST.radians, abs_tol=self._cardinal_tolerance)):
            return True
        else:
            return False

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def difference_from_cardinal(self, cardinal):
        '''
        Returns the difference between the current heading and the provided
        Cardinal direction, in radians.
        '''
        return Convert.get_offset_from_cardinal(self.heading_radians, cardinal)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def ratio_from_cardinal(self, cardinal):
        '''
        Returns a ratio (range: 0.0-1.0) between the current heading and the
        provided Cardinal direction.
        '''
        return Convert.get_offset_from_cardinal(self.heading_radians, cardinal) / HALF_PI

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable_displays(self):
        self._show_rgbmatrix5x5  = False
        self._show_rgbmatrix11x7 = False

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def pitch(self):
        '''
        Return the last-polled pitch value.
        '''
        return self._pitch

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def roll(self):
        '''
        Return the last-polled roll value.
        '''
        return self._roll


    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def uncalibrated_heading(self):
        '''
        Return the compass heading in degrees from a potentially
        uncalibrated IMU.

        This may not be a valid value if the device is not calibrated.
        '''
        if self._amin is None or self._amax is None:
            self._amin = list(self.__icm20948.read_magnetometer_data())
            self._amax = list(self.__icm20948.read_magnetometer_data())
        return self._read_heading(self._amin, self._amax)
#       return self._heading

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def heading(self):
        '''
        Return the last-polled compass heading in degrees (as an int).

        This is only valid if the device is calibrated.
        '''
        return self._heading

    @property
    def heading_radians(self):
        '''
        Return the last-polled compass heading in radians.

        This is only valid if the device is calibrated.
        '''
        return self._radians

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_formatted_heading(self):
        '''
        Return a lambda function whose result is the last-polled compass
        heading in degrees, formatted as a string.

        This is only valid if the device is calibrated.
        '''
        return self._formatted_heading

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def mean_heading(self):
        '''
        Return the mean compass heading in degrees (as an int). This is the
        mean value of the current queue, whose size and rate accumulated are
        set in configuration. Because this is calculated from the queue, if
        the queue is changing rapidly this returned value won't accurately
        reflect the mean. Depending on configuration this takes roughly 1
        second to stabilise to a mean reflective of the robot's position,
        which then doesn't change very quickly. It is therefore suitable for
        gaining an accurate heading of a resting robot.

        This is only valid if the device is calibrated.
        '''
        return self._mean_heading

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def mean_heading_radians(self):
        '''
        Return the mean compass heading in radians (as a float).
        '''
        return self._mean_heading_radians

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def standard_deviation(self):
        '''
        Return the current value of the standard deviation of headings
        calculated from the queue.
        '''
        return self._stdev

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def accelerometer(self):
        '''
        Return the IMU's accelerometer value as an x,y,z value.
        If not enabled this returns zeros.
        '''
        return self._accel

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def gyroscope(self):
        '''
        Return the IMU's gyroscope value as an x,y,z value.
        If not enabled this returns zeros.
        '''
        return self._gyro

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def calibrate(self):
        '''
        Manually calibrate the sensor by looping while the sensor is rotated
        through a 360° motion, then leave it to rest for a few seconds. This
        times out after 60 seconds.

        There is a ballistic behaviour in MotionController to perform this
        same function.

        Returns True or False upon completion (in addition to setting the
        class variable).
        '''
        _start_time = dt.now()
        self._heading_count = 0
        _rate = Rate(self._poll_rate_hz, Level.ERROR)
        _ranger = Ranger(0.0, 180.0, 0.0, 0.5)
        _counter = itertools.count()
        _count = 0
        _limit = 1800 # 1 minute
        self._amin = list(self.__icm20948.read_magnetometer_data())
        self._amax = list(self.__icm20948.read_magnetometer_data())
        if self._play_sound:
            self._player.play(Sound.CHATTER_2)
        self._log.info(Fore.WHITE + Style.BRIGHT + '\n\n    calibrate by rotating sensor through a horizontal 360° motion…\n')
        if self._show_rgbmatrix5x5 and self._rgbmatrix5x5:
            self._rgbmatrix.set_display_type(DisplayType.RANDOM)
            self._rgbmatrix.enable()
            self._rgbmatrix.set_random_delay_sec(_ranger.convert(180.0)) # speeds up random display as stdev shrinks
        while True:
            _count = next(_counter)
            if self.is_calibrated or _count > _limit:
                break
            try:
                _heading = self._read_heading(self._amin, self._amax)
                r, g, b = [int(c * 255.0) for c in hsv_to_rgb(_heading / 360.0, 1.0, 1.0)]
                if self.calibration_check(_heading):
                    break
                if self._rgbmatrix5x5 and _count % 10 == 0:
                    self._rgbmatrix.set_random_delay_sec(_ranger.convert(self._stdev)) # speeds up random display as stdev shrinks
                    self._log.info(Fore.CYAN + '[{:d}] trying to calibrate… stdev: {:4.2f}; '.format(_count, self._stdev) + Style.DIM + '(calibrated? {}; over limit? {})'.format(
                            self.is_calibrated, _count > _limit))
            except Exception as e:
                self._log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
            _rate.wait()

        _elapsed_ms = round(( dt.now() - _start_time ).total_seconds() * 1000.0)
        if self._show_rgbmatrix5x5 and self._rgbmatrix5x5:
            self._rgbmatrix.set_display_type(DisplayType.DARK)
            self._rgbmatrix.disable()
        if self.is_calibrated:
            self._log.info(Fore.GREEN + 'IMU calibrated: elapsed: {:d}ms'.format(_elapsed_ms))
            if self._play_sound:
                self._player.play(Sound.CHATTER_4)
        else:
            self._log.error('unable to calibrate IMU after elapsed: {:d}ms'.format(_elapsed_ms))
        return self.is_calibrated

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def clear_queue(self):
        '''
        Clears the statistic queue.
        '''
        self._queue.clear()
#       for _ in range(100): # ...by populating it with zeros.
#           self._queue.append(0)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def calibration_check(self, heading):
        '''
        Adds a heading value to the queue and checks to see if the IMU is
        calibrated, according to the contents of the queue having a standard
        deviation less than a set threshold.

        Note that this does not clear the queue.
        '''
        self._queue.append(heading)
        self._heading_count += 1
        if len(self._queue) < self._queue_length: # we only calibrate after the queue is full
            return False
        self._stdev = statistics.stdev(self._queue)
#       self._log.info('added heading of {:4.2f} to queue of {:d} values in queue with stdev of: {:5.3f}.'.format(heading, self._heading_count, self._stdev))
        if self._stdev < self._stability_threshold: # stable? then permanently flag as calibrated
            self.set_is_calibrated(True)
        return self._is_calibrated

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def scan(self, enabled=None, callback=None):
        '''
        This starts a loop that will repeat until the application exits or
        until the optional enabled flag becomes False. For a single read of
        the sensor use poll().

        The optional callback will be executed upon each loop.

        Note: calling this method will fail if not previously calibrated.
        '''
        _rate = Rate(self._poll_rate_hz, Level.ERROR)
        if self._amin is None or self._amax is None:
            raise Exception('compass not calibrated yet, call calibrate() first.')
        while enabled():
            if self._adjust_trim:
                self._trim = self._digital_pot.get_scaled_value(False)
                if self._show_rgbmatrix5x5:
                    self._digital_pot.set_rgb(self._digital_pot.value)
            self.poll()
            if callback:
                callback()
            _rate.wait()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def poll(self):
        '''
        An individual call to the sensor. This is called in a loop by scan(),
        but can be called independently. This sets heading, pitch and roll.

        Note: calling this method will fail if not previously calibrated.
        '''
        if not self.is_calibrated:
            raise Exception('IMU is not calibrated.')
        try:
            self._heading = self._read_heading(self._amin, self._amax)
            # add to queue to calculate mean heading
            self._queue.append(self._heading)
            self._stdev = statistics.stdev(self._queue)
            if self._stdev < self._stability_threshold: # stable? then permanently flag as calibrated
                self.set_is_calibrated(True)
            self._mean_heading = statistics.mean(self._queue)
            self._mean_heading_radians = math.radians(self._mean_heading)
            if next(self._counter) % self._display_rate == 0: # display every 10th set of values
                # convert to RGB
                r, g, b = [int(c * 255.0) for c in hsv_to_rgb(self._heading / 360.0, 1.0, 1.0)]
                if self._show_console:
                    if self._is_calibrated:
                        _style = Style.BRIGHT
                    else:
                        _style = Style.NORMAL
                    self._log.info(_style + "heading: {:3d}° / mean: {:3d}°;".format(self._heading, int(self._mean_heading))
                            + Style.NORMAL + " stdev: {:.2f}; trim: {:.2f}; color: #{:02X}{:02X}{:02X}".format(
                                self._stdev, self._trim, r, g, b))
#                   if self._include_accel_gyro:
#                       self._log.info(Fore.WHITE + "accel: {:5.2f}, {:5.2f}, {:5.2f}; gyro: {:5.2f}, {:5.2f}, {:5.2f}".format(*self._accel, *self._gyro))
                if self._show_rgbmatrix5x5 and self._rgbmatrix5x5:
                    if self._is_calibrated:
                        RgbMatrix.set_all(self._rgbmatrix5x5, r, g, b)
                        if self.is_cardinal_aligned():
                            self._rgbmatrix5x5.set_brightness(0.8)
                        else:
                            self._rgbmatrix5x5.set_brightness(0.3)
                    else:
                        RgbMatrix.set_all(self._rgbmatrix5x5, 40, 40, 40)
                    self._rgbmatrix5x5.show()
                if self._show_rgbmatrix11x7:
                    self._matrix11x7.clear()
                    self._matrix11x7.write_string('{:>3}'.format(self._heading), y=1, font=font3x5)
                    self._matrix11x7.show()
                    if self._is_calibrated:
                        self._matrix11x7.set_brightness(self._high_brightness)
#                       self._matrix11x7.set_brightness(self._medium_brightness)
                    else:
                        self._matrix11x7.set_brightness(self._low_brightness)
            # now get pitch and roll ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            z, x, y = self.accelerometer # note Z,X,Y based on orientation of installed IMU
            self._pitch = ( -180.0 * math.atan(x/math.sqrt(y*y + z*z)) / math.pi ) + self._pitch_trim
            self._roll  = ( -180.0 * math.atan(y/math.sqrt(x*x + z*z)) / math.pi ) + self._roll_trim
            return self._heading, self._pitch, self._roll
        except Exception as e:
            self._log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
            return None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _read_heading(self, amin, amax):
        '''
        Does the work of obtaining the heading value in degrees.
        '''
        mag = list(self.__icm20948.read_magnetometer_data())
        if self._include_accel_gyro:
            # ax, ay, az, gx, gy, gz
            self._accel[0], self._accel[1], self._accel[2], self._gyro[0], self._gyro[1], self._gyro[2] = self.__icm20948.read_accelerometer_gyro_data()
        for i in range(3):
            v = mag[i]
            # if our current reading (mag) is less than our stored minimum
            # reading (amin), then save a new minimum reading, i.e., save a
            # new lowest possible value for our calibration of this axis
            if v < amin[i]:
                amin[i] = v
            # if our current reading (mag) is greater than our stored maximum
            # reading (amax), then save a new maximum reading, i.e., save a
            # new highest possible value for our calibration of this axis
            if v > amax[i]:
                amax[i] = v
            # calibrate value by removing any offset when compared to the
            # lowest reading seen for this axes
            mag[i] -= amin[i]
            # scale value based on the highest range of values seen for this
            # axes. Creates a calibrated value between 0 and 1 representing
            # magnetic value
            try:
                mag[i] /= amax[i] - amin[i]
            except ZeroDivisionError:
                pass
            # Shift magnetic values to between -0.5 and 0.5 to enable the trig to work
            mag[i] -= 0.5
        # convert from Gauss values in the appropriate 2 axis to a heading
        # in Radians using trig. Note this does not compensate for tilt.
        self._radians = math.atan2(mag[self._axes[0]],mag[self._axes[1]])
        # add potentiometer trim (in radians, ±1𝛑)
        self._radians += self._trim
         # if heading is negative, convert to positive, 2 x pi is a full circle in Radians
        if self._radians < 0:
            self._radians += 2 * math.pi
#       _raw_radians = self._radians
        # convert heading from radians to degrees
        _degrees = math.degrees(self._radians)
        # round heading to nearest full degree
        return int(round(_degrees))

#EOF
