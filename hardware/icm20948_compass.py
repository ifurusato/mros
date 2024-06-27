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
from collections import deque
from datetime import datetime as dt
from colorsys import hsv_to_rgb
from colorama import init, Fore, Style
init()

from icm20948 import ICM20948
from rgbmatrix5x5 import RGBMatrix5x5
from matrix11x7 import Matrix11x7
from matrix11x7.fonts import font3x5, font5x5, font5x7, font5x7smoothed

from core.component import Component
from core.logger import Logger, Level
from core.rate import Rate
from core.config_loader import ConfigLoader
from hardware.i2c_scanner import I2CScanner, DeviceNotFound
from hardware.digital_pot import DigitalPotentiometer
from hardware.rgbmatrix import RgbMatrix
from hardware.sound import Player, Sound

IN_MIN  = 0.0  # minimum analog value from IO Expander
IN_MAX  = 3.3  # maximum analog value from IO Expander
OUT_MIN = -1.0 * math.pi # minimum scaled output value
OUT_MAX = math.pi        # maximum scaled output value

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Icm20948(Component):
    NORTH_RADIANS = math.pi * 2.0
    WEST_RADIANS  = math.pi / 2.0
    SOUTH_RADIANS = math.pi
    EAST_RADIANS  = math.pi * 1.5
    '''
    Wraps the functionality of an ICM20948 IMU as a compass. This includes
    optional trim adjustment, a calibration check, and optional console,
    numeric and color displays of heading, as well as making accelerometer
    and gyroscope values available.

    :param config:          the application configuration
    :param rgbmatrix        the optional RGB matrix to indicate calibration
    :param level            the log level
    '''
    def __init__(self, config, rgbmatrix=None, level=Level.INFO):
        self._log = Logger('compass', level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._log.info('initialising compass…')
        if not isinstance(config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(name)))
        # add color display ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._rgbmatrix          = rgbmatrix
        if self._rgbmatrix:
            if not isinstance(self._rgbmatrix, RGBMatrix5x5):
                raise ValueError('wrong type for RGB matrix argument: {}'.format(type(self._rgbmatrix)))
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _cfg = config['mros'].get('hardware').get('icm20948')
        self._adjust_trim        = _cfg.get('adjust_trim')
        self._show_console       = _cfg.get('show_console')
        self._show_rgbmatrix11x7 = _cfg.get('show_rgbmatrix11x7')
        self._play_sound         = _cfg.get('play_sound') # if True, play sound to indicate calibration
        if self._play_sound:
            self._player = Player.instance()
        else:
            self._player = None
        # set up trim control ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._trim = 0.0
        if not self._adjust_trim:
            # use fixed value
            self._trim = _cfg.get('trim')
        self._pot = None
        if self._adjust_trim:
            # configure potentiometer ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
            _i2c_scanner = I2CScanner(config, level)
            if _i2c_scanner.has_hex_address(['0x0E']):
                self._log.info('using digital potentiometer…')
                _min_value = -100.0
                _max_value = 100.0
                self._pot = DigitalPotentiometer(config, level=level)
                self._pot.set_input_range(IN_MIN, IN_MAX)
                self._pot.set_output_range(OUT_MIN, OUT_MAX)
#           else:
#               raise Exception('no digital potentiometer available.')
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
        self._stdev = 999
        _queue_len = _cfg.get('queue_length') # also affects how fast mean catches up to data
        self._queue = deque(_queue_len*[0], _queue_len)
        self._stability_threshold = _cfg.get('stability_threshold')
        # misc/variables ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._display_rate = 20 # display every 10th set of values
        self._poll_rate_hz = _cfg.get('poll_rate_hz')
        self._amin = None
        self._amax = None
        self._heading = 0
        self._mean_heading = 0
        self._mean_heading_radians = 0.0
        self._accel = [0.0, 0.0, 0.0]
        self._gyro =  [0.0, 0.0, 0.0]
        self._include_accel_gyro = _cfg.get('include_accel_gyro')
        self._is_calibrated = False
        # instantiate sensor class  ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._imu = ICM20948(i2c_addr=_cfg.get('i2c_address'))
        self._log.info('ready.')

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

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def is_cardinal_aligned(self):
        '''
        Returns True if the mean heading is aligned within a 3° tolerance to
        one of the four cardinal directions.
        '''
        if math.isclose(self._mean_heading_radians, Icm20948.NORTH_RADIANS, abs_tol=self._cardinal_tolerance):
            return True
        elif math.isclose(self._mean_heading_radians, Icm20948.WEST_RADIANS, abs_tol=self._cardinal_tolerance):
            return True
        elif math.isclose(self._mean_heading_radians, Icm20948.SOUTH_RADIANS, abs_tol=self._cardinal_tolerance):
            return True
        elif math.isclose(self._mean_heading_radians, Icm20948.EAST_RADIANS, abs_tol=self._cardinal_tolerance):
            return True
        else:
            return False

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def heading(self):
        '''
        Return the compass heading in degrees (as an int).

        This is only valid if the device is calibrated.
        '''
        return self._heading

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

        Returns True or False upon completion (in addition to setting the
        class variable).
        '''
        _start_time = dt.now()
        _rate = Rate(self._poll_rate_hz, Level.ERROR)
        _counter = itertools.count()
        _count = 0
        _limit = 1800 # 1 minute
        self._amin = list(self._imu.read_magnetometer_data())
        self._amax = list(self._imu.read_magnetometer_data())
        self._log.info(Fore.WHITE + Style.BRIGHT + '\n    calibrate by rotating sensor through a horizontal 360° motion…\n' + Style.RESET_ALL)
        if self._rgbmatrix:
            RgbMatrix.set_all(self._rgbmatrix, 40, 40, 40)
            self._rgbmatrix.show()
        while True:
            _count = next(_counter)
            if self.is_calibrated or _count > _limit:
                break
            try:
                _heading = self._read_heading(self._amin, self._amax)
                r, g, b = [int(c * 255.0) for c in hsv_to_rgb(_heading / 360.0, 1.0, 1.0)]
                # add to queue
                self._queue.append(_heading)
                _stdev = statistics.stdev(self._queue)
                if _stdev < self._stability_threshold: # stable? then permanently flag as calibrated
                    self._is_calibrated = True
                    break
                if _count % 50 == 0:
                    self._log.info(Fore.CYAN + '[{:d}] trying to calibrate… '.format(_count) + Style.DIM + '(calibrated? {}; over limit? {})'.format(
                            self.is_calibrated, _count > _limit) + Style.RESET_ALL)
            except Exception as e:
                self._log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))
            _rate.wait()

        _elapsed_ms = round(( dt.now() - _start_time ).total_seconds() * 1000.0)
        if self._rgbmatrix:
            RgbMatrix.set_all(self._rgbmatrix, 0, 0, 0)
            self._rgbmatrix.show()
        if self.is_calibrated:
            self._log.info(Fore.GREEN + 'IMU calibrated: elapsed: {:d}ms'.format(_elapsed_ms))
            if self._play_sound:
                self._player.play(Sound.CHATTER_4)
        else:
            self._log.error('unable to calibrate IMU after elapsed: {:d}ms'.format(_elapsed_ms))
        return self.is_calibrated

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def scan(self):
        '''
        Note: calling this method will fail if not previously calibrated.
        '''
        _rate = Rate(self._poll_rate_hz, Level.ERROR)
        _counter = itertools.count()
        if self._amin is None or self._amax is None:
            raise Exception('compass not calibrated yet, call calibrate() first.')
        while True:
            if self._adjust_trim:
                self._trim = self._pot.get_scaled_value(False)
                if self._show_rgbmatrix5x5:
                    self._pot.set_rgb(self._pot.value)
            self.poll(_counter)
            _rate.wait()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def poll(self, counter):
        '''
        An individual call to the sensor. This is called in a loop by scan(),
        but can be called independently.

        Note: calling this method will fail if not previously calibrated.
        '''
        try:
            self._heading = self._read_heading(self._amin, self._amax)
            # add to queue to calculate mean heading
            self._queue.append(self._heading)
            _stdev = statistics.stdev(self._queue)
            if _stdev < self._stability_threshold: # stable? then permanently flag as calibrated
                self._is_calibrated = True
            self._mean_heading = statistics.mean(self._queue)
            self._mean_heading_radians = math.radians(self._mean_heading)
            if next(counter) % self._display_rate == 0: # display every 10th set of values
                # convert to RGB
                r, g, b = [int(c * 255.0) for c in hsv_to_rgb(self._heading / 360.0, 1.0, 1.0)]
                if self._show_console:
                    if self._is_calibrated:
                        _style = Style.BRIGHT
                    else:
                        _style = Style.NORMAL
                    self._log.info(_style + "heading: {:3d}° / mean: {:3d}°;".format(self._heading, int(self._mean_heading))
                            + Style.NORMAL + " stdev: {:.2f}; trim: {:.2f}; color: #{:02X}{:02X}{:02X}".format(
                                _stdev, self._trim, r, g, b) + Style.RESET_ALL)
#                   if self._include_accel_gyro:
#                       self._log.info(Fore.WHITE + "accel: {:5.2f}, {:5.2f}, {:5.2f}; gyro: {:5.2f}, {:5.2f}, {:5.2f}".format(*self._accel, *self._gyro) + Style.RESET_ALL)
                if self._rgbmatrix:
                    if self._is_calibrated:
                        RgbMatrix.set_all(self._rgbmatrix, r, g, b)
                        if self.is_cardinal_aligned():
                            self._rgbmatrix.set_brightness(0.8)
                        else:
                            self._rgbmatrix.set_brightness(0.3)
                    else:
                        RgbMatrix.set_all(self._rgbmatrix, 40, 40, 40)
                    self._rgbmatrix.show()
                if self._show_rgbmatrix11x7:
                    self._matrix11x7.clear()
                    self._matrix11x7.write_string('{:>3}'.format(self._heading), y=1, font=font3x5)
                    self._matrix11x7.show()
                    if self._is_calibrated:
                        self._matrix11x7.set_brightness(self._high_brightness)
#                       self._matrix11x7.set_brightness(self._medium_brightness)
                    else:
                        self._matrix11x7.set_brightness(self._low_brightness)

        except Exception as e:
            self._log.error('{} encountered, exiting: {}\n{}'.format(type(e), e, traceback.format_exc()))


    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _read_heading(self, amin, amax):
        '''
        Does the work of obtaining the heading value in degrees.
        '''
        mag = list(self._imu.read_magnetometer_data())
        if self._include_accel_gyro:
            # ax, ay, az, gx, gy, gz
            self._accel[0], self._accel[1], self._accel[2], self._gyro[0], self._gyro[1], self._gyro[2] = self._imu.read_accelerometer_gyro_data()
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
        _radians = math.atan2(mag[self._axes[0]],mag[self._axes[1]])
        # add potentiometer trim (in radians, ±1π)
        _radians += self._trim
         # if heading is negative, convert to positive, 2 x pi is a full circle in Radians
        if _radians < 0:
            _radians += 2 * math.pi
#       _raw_radians = _radians
        # convert heading from radians to degrees
        _degrees = math.degrees(_radians)
        # round heading to nearest full degree
        return int(round(_degrees))

#EOF
