#!/usr/bin/env python3

# -*- coding: utf-8 -*-
#
# Portions copyright 2020-2024 by Murray Altheim. All rights reserved. This file
# is part of the Robot Operating System project, released under the MIT License.
# Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-09-03
# modified: 2024-09-04
#
#  The Em7180 class is used for running the USFS SENtral sensor hub as an IMU.
#
# See:       https://github.com/simondlevy/USFS
# See:       https://github.com/simondlevy/USFS/tree/master/examples/WarmStartAndAccelCal
# See also:  https://github.com/kriswiner/EM7180_SENtral_sensor_hub/tree/master
# Source:    https://github.com/simondlevy/USFS/blob/master/python/mastertest.py
#
'''
   mastertest.py: Example Python script for running USFS SENtral sensor
   hub in master mode.

   Copyright (C) 2018 Simon D. Levy

   This file is part of USFS.

   USFS is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   USFS is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with USFS.  If not, see <http://www.gnu.org/licenses/>.
'''

from usfs import USFS_Master

import math
from colorama import init, Fore, Style
init()

from core.component import Component
from core.logger import Logger, Level
from matrix11x7.fonts import font3x5

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Em7180(Component):

    # constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    MAG_RATE       = 100  # Hz
    ACCEL_RATE     = 200  # Hz
    GYRO_RATE      = 200  # Hz
    BARO_RATE      = 50   # Hz
    Q_RATE_DIVISOR = 3    # 1/3 gyro rate
    '''
    Wraps the functionality of the Pesky Products EM7180 sensor hub with an
    Invensense MPU9250 gyro/accelerometer and Asahi Kasei AK8963C magnetometer,
    providing options for displaying the heading on an 11x7 LED matrix, and an
    option for providing a digital potentiometer for adjusting the heading (yaw)
    trim.

    :param: config     application configuration
    :param: config     optional 11x7 matrix to display heading
    :param: trim_pot   optional digital potentiometer to set magnetometer trim
    :param: level      log level
    '''
    def __init__(self, config, matrix11x7=None, trim_pot=None, level=Level.INFO):
        self._log = Logger('usfs', level)
        Component.__init__(self, self._log, suppressed=False, enabled=False)
        self._log.info('initialising usfs…')
        if not isinstance(config, dict):
            raise ValueError('wrong type for config argument: {}'.format(type(name)))
        self._matrix11x7 = matrix11x7
        self._trim_pot = trim_pot
        # create USFS ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self._usfs = USFS_Master(self.MAG_RATE, self.ACCEL_RATE, self.GYRO_RATE, self.BARO_RATE, self.Q_RATE_DIVISOR)
        # start the USFS in master mode ┈┈┈┈┈┈┈┈┈┈
        if not self._usfs.begin():
            self._log.error('unable to start USFS: {}'.format(self._usfs.getErrorString()))
#           sys.exit(1)
            self.close()
        self._use_matrix    = matrix11x7 != None
        self._verbose       = True # if true display to console
        self._pitch         = 0.0
        self._roll          = 0.0
        self._yaw           = 0.0
        self._corrected_yaw = 0.0
        self._yaw_trim      = 0.0
        self._pressure      = 0.0
        self._temperature   = 0.0
        self._altitude      = 0.0
        self._ax = self._ay = self._az = 0.0
        self._gx = self._gy = self._gz = 0.0
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_verbose(self, verbose):
        self._verbose = verbose

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def pitch(self):
        '''
        After calling poll(), this returns the latest pitch value.
        '''
        return self._pitch

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def roll(self):
        '''
        After calling poll(), this returns the latest roll value.
        '''
        return self._roll

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def yaw(self):
        '''
        After calling poll(), this returns the latest uncorrected yaw value.
        '''
        return self._yaw

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def corrected_yaw(self):
        '''
        After calling poll(), this returns the latest corrected (trimmed)
        yaw value.
        '''
        return self._corrected_yaw

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def yaw_trim(self):
        '''
        Returns the current yaw trim value as set by the digital potentiometer.
        '''
        return self._yaw_trim

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def pressure(self):
        '''
        After calling poll(), this returns the latest pressure value.
        '''
        return self._pressure

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def temperature(self):
        '''
        After calling poll(), this returns the latest temperature value.
        '''
        return self._temperature

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def altitude(self):
        '''
        After calling poll(), this returns the latest altitude value.
        '''
        return self._altitude

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def accelerometer(self):
        '''
        After calling poll(), this returns the x,y,z tuple value from the
        accelerometer.
        '''
        return self._ax, self._ay, self._az

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def gyroscope(self):
        '''
        After calling poll(), this returns the x,y,z tuple value from the
        gyroscope.
        '''
        return self._gx, self._gy, self._gz

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def poll(self):

        self._usfs.checkEventStatus()
        if self._usfs.gotError():
            self._log.error('error starting USFS: {}'.format(self._usfs.getErrorString()))
#           sys.exit(1)
            self.close()

        # Define output variables from updated quaternion---these are Tait-Bryan
        # angles, commonly used in aircraft orientation.  In this coordinate
        # system, the positive z-axis is down toward Earth.  Yaw is the angle
        # between Sensor x-axis and Earth magnetic North (or true North if
        # corrected for local declination, looking down on the sensor positive
        # yaw is counterclockwise.  Pitch is angle between sensor x-axis and
        # Earth ground plane, toward the Earth is positive, up toward the sky is
        # negative.  Roll is angle between sensor y-axis and Earth ground plane,
        # y-axis up is positive roll.  These arise from the definition of the
        # homogeneous rotation matrix constructed from q.  Tait-Bryan
        # angles as well as Euler angles are non-commutative that is, the get
        # the correct orientation the rotations must be applied in the correct
        # order which for this configuration is yaw, pitch, and then roll.  For
        # more see http://en.wikipedia.org/wiki/Conversion_between_q_and_Euler_angles
        # which has additional links.

        if (self._usfs.gotQuaternion()):

            qw, qx, qy, qz = self._usfs.readQuaternion()

            self._roll  = math.atan2(2.0 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
            self._pitch = -math.asin(2.0 * (qx * qz - qw * qy))
            self._yaw   = math.atan2(2.0 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz)

            self._pitch *= 180.0 / math.pi
            self._yaw   *= 180.0 / math.pi
            self._yaw   += 13.8 # Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
            if self._yaw < 0: self._yaw   += 360.0  # Ensure yaw stays between 0 and 360
            self._roll  *= 180.0 / math.pi

    #       print('Quaternion Roll, Pitch, Yaw: %+2.2f %+2.2f %+2.2f' % (roll, pitch, yaw))

            self._yaw_trim = self._trim_pot.get_scaled_value()
            self._corrected_yaw = self._yaw - self._yaw_trim
            # ensure yaw stays between 0 and 360
            if self._corrected_yaw < 0: self._corrected_yaw += 360.0
            elif self._corrected_yaw > 360: self._corrected_yaw -= 360.0
            if self._verbose:
                self._log.info('Quaternion Roll: {:+2.2f}; Pitch: {:+2.2f}; '.format(self._roll, self._pitch)
                        + Fore.BLUE + 'Yaw: {:+2.2f} '.format(self._yaw)
                        + Fore.WHITE + 'Corrected Yaw: {:+2.2f} '.format(self._corrected_yaw)
                        + Style.DIM + 'with trim: {:+2.2f}'.format(self._yaw_trim))

            if self._use_matrix:
                self._matrix11x7.clear()
                self._matrix11x7.write_string('{:>3}'.format(int(self._corrected_yaw)), y=1, font=font3x5)
                self._matrix11x7.show()

        if self._usfs.gotAccelerometer():
            self._ax, self._ay, self._az = self._usfs.readAccelerometer()
            if self._verbose:
                self._log.info('Accel: {:+3.3f} {:+3.3f} {:+3.3f}'.format(self._ax, self._ay, self._az))

        if self._usfs.gotGyrometer():
            self._gx, self._gy, self._gz = self._usfs.readGyrometer()
            if self._verbose:
                self._log.info('Gyro: {:+3.3f} {:+3.3f} {:+3.3f}'.format(self._gx,self._gy,self._gz))

         #  Or define output variable according to the Android system, where
         #  heading (0 to 360) is defined by the angle between the y-axis and True
         #  North, pitch is rotation about the x-axis (-180 to +180), and roll is
         #  rotation about the y-axis (-90 to +90) In this systen, the z-axis is
         #  pointing away from Earth, the +y-axis is at the 'top' of the device
         #  (cellphone) and the +x-axis points toward the right of the device.

        if self._usfs.gotBarometer():
            self._pressure, self._temperature = self._usfs.readBarometer()
            self._altitude = (1.0 - math.pow(self._pressure / 1013.25, 0.190295)) * 44330
            if self._verbose:
                self._log.info('Baro:')
                self._log.info('  Altimeter temperature = {:+2.2f} C'.format(self._temperature))
                self._log.info('  Altimeter pressure = {:+2.2f} mbar'.format(self._pressure))
                self._log.info('  Altitude = {:+2.2f} m\n'.format(self._altitude))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        if self.closed:
            self._log.warning('cannot enable USFS: already closed.')
        else:
            if self.enabled:
                self._log.warning('USFS already enabled.')
            else:
                Component.enable(self)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        Component.disable(self)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Closes the USFS, calling disable.
        '''
        Component.close(self)

#EOF
