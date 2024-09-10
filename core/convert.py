#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-03-27
# modified: 2024-08-26
#
#  A class containing some static IMU-related conversion methods.
#

import numpy, math
from math import pi as π
from math import tau as τ

from core.cardinal import Cardinal

X = 0
Y = 1
Z = 2
AXES = Y, Z

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class Convert:

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def to_degrees(radians):
        return math.degrees(radians)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def to_radians(degrees):
        return math.radians(degrees)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def rps_to_dps(rps):
        return rps * 57.29578

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def offset_in_degrees(angle, offset):
        '''
        Add two angles (provided in degrees), returning the result.
        '''
        return ( angle + offset ) % 360.0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def difference_in_degrees(starting_angle, target_angle):
        '''
        Returns the shortest angular difference between two angles (provided
        in degrees), returning the result as an int.
        '''
        _starting_angle_rad = math.radians(starting_angle)
        _target_angle_rad   = math.radians(target_angle)
        return round(math.degrees(math.atan2(math.sin(_target_angle_rad - _starting_angle_rad), math.cos(_target_angle_rad - _starting_angle_rad))))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def difference_in_radians(starting_angle, target_angle):
        '''
        Return the shortest angular difference between two angles provided
        in radians, returning the result as a float. Uses trigonometry.
        '''
        return math.atan2(math.sin(target_angle - starting_angle), math.cos(target_angle - starting_angle))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_angle_between_in_radians(a1, a2):
        '''
        Computes the shortest angular difference between the two angles
        provided in radians, returning the value as a float. Uses geometry.
        '''
        return min(a2 - a1, a2 - a1 + τ, a2 - a1 - τ, key=abs)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_offset_from_cardinal(radians, cardinal):
         '''
         Returns the ± difference in angle between the argument
         and the provided Cardinal direction.
         '''
         if not isinstance(radians, float):
             raise ValueError('expected float value for radians argument.')
         if not isinstance(cardinal, Cardinal):
             raise ValueError('expected Cardinal value for cardinal argument.')
         return Convert.get_angle_between_in_radians(radians, cardinal.radians)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def quaternion_to_euler_angle(w, x, y, z):
        q = Quaternion(w, x, y, z)
        deg = q.degrees
        return deg

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def quaternion_to_euler(w, x, y, z):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        heading = math.atan2(t3, t4)
        return [heading, pitch, roll]

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def heading_from_magnetometer(amin, amax, mag, offset):
        '''
        :param amin:     the original list of magnetometer readings used as a minimum
        :param amax:     the original list of magnetometer readings used as a maximum
        :param mag:      the magnetometer reading (x, y, z) to convert to a heading
        :param offset:   the optional offset in degrees
        '''
        mag = list(mag)
        for i in range(3):
            v = mag[i]
            if v < amin[i]:
                amin[i] = v
            if v > amax[i]:
                amax[i] = v
            mag[i] -= amin[i]
            try:
                mag[i] /= amax[i] - amin[i]
            except ZeroDivisionError:
                pass
            mag[i] -= 0.5

        heading_rad = math.atan2(mag[AXES[0]], mag[AXES[1]])
        if heading_rad < 0:
            heading_rad += τ
        heading_deg = math.degrees(heading_rad)
        if offset != 0:
            heading_deg = Convert.offset_in_degrees(heading_deg, offset)
        heading_deg = int(round(heading_deg))
        return heading_deg

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def quaternion_to_euler_angle_other(w, x, y, z):
        ysqr = y * y
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = numpy.degrees(numpy.arctan2(t0, t1))
        t2 = +2.0 * (w * y - z * x)
        t2 = numpy.clip(t2, a_min=-1.0, a_max=1.0)
        Y = numpy.degrees(numpy.arcsin(t2))
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = numpy.degrees(numpy.arctan2(t3, t4))
        return X, Y, Z

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def convert_to_euler(qw, qx, qy, qz):
        # can get the euler angles back out in degrees (set to True)
        _euler = quat2euler(qw, qx, qy, qz, degrees=True)
        _heading = -1.0 * _euler[2]
        _pitch   = _euler[1]
        _roll    = -1.0 * _euler[0]
        return [ _heading, _pitch, _roll ]

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def convert_to_degrees(x, y, z):
        '''
        Provided a x,y,z magnetometer reading returns a heading value in degrees.

        source:  https://cdn-shop.adafruit.com/datasheets/AN203_Compass_Heading_Using_Magnetometers.pdf
        '''
        if y == 0.0:
            if x < 0.0:
                return 180.0
            elif x >= 0.0:
                return 0.0
        elif y > 0.0:
            return 90.0 - math.atan( x / y ) * ( 180.0 / π )
        elif y < 0.0:
            return 270.0 - math.atan( x / y ) * ( 180.0 / π )

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def rotate_90_degrees(degrees):
        '''
        Return the argument rotated 90 degrees.
        '''
        return Convert.to_degrees(Convert.to_radians(degrees) - ( π / 2.0 ))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def rotate_180_degrees(degrees):
        '''
        Return the argument rotated 180 degrees.
        '''
        return Convert.to_degrees((Convert.to_radians(degrees) + π) % τ)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def in_range(p, q, error_range):
        '''
        Returns True if the first two numbers are within the supplied range
        of each other.
        '''
        return p >= ( q - error_range ) and p <= ( q + error_range )

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def convert_to_distance(value):
        '''
        Converts the value returned by the IR sensor to a distance in centimeters.

        Distance Calculation ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

        This is reading the distance from a 3 volt Sharp GP2Y0A60SZLF infrared
        sensor to a piece of white A4 printer paper in a low ambient light room.
        The sensor output is not linear, but its accuracy is not critical. If
        the target is too close to the sensor the values are not valid. According
        to spec 10cm is the minimum distance, but we get relative variability up
        until about 5cm. Values over 150 clearly indicate the robot is less than
        10cm from the target. Here's a sampled output:

            0cm = unreliable
            5cm = 226.5
          7.5cm = 197.0
           10cm = 151.0
           20cm =  92.0
           30cm =  69.9
           40cm =  59.2
           50cm =  52.0
           60cm =  46.0
           70cm =  41.8
           80cm =  38.2
           90cm =  35.8
          100cm =  34.0
          110cm =  32.9
          120cm =  31.7
          130cm =  30.7 *
          140cm =  30.7 *
          150cm =  29.4 *

        * Maximum range on IR is about 130cm, after which there is diminishing
          stability/variability, i.e., it's hard to determine if we're dealing
          with a level of system noise rather than data. Different runs produce
          different results, with values between 28 - 31 on a range of any more
          than 130cm.

        See: http://ediy.com.my/blog/item/92-sharp-gp2y0a21-ir-distance-sensors
        '''
        if value == None or value == 0:
            return None
        _FUDGE_FACTOR = -2.00
        _EXPONENT = 1.34
        _NUMERATOR = 1000.0
        _distance = pow( _NUMERATOR / value, _EXPONENT ) + _FUDGE_FACTOR
        return _distance

# EOF
