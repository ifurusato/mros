#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-07-07
# modified: 2024-06-07
#

import time
import json
from micropython import const
from pyb import RTC
from colorama import Fore, Style

rtc = RTC()

_ISO_FORMAT_STRING = const("{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}.{:03d}Z")

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Util(object):
    '''
    A collection of static utility methods.
    '''
    def __init__(self):
        super().__init__()
        self._log = Logger('util', level)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def is_true(value):
        '''
        Returns True if the value is a 1, a "1", "y", "yes", or "true" (with
        case-insensitive matching).
        '''
        if value:
            if isinstance(value, int):
                return value == 1
            else:
                _value = value.lower()
                return _value == "1" or _value == "y" or _value == "yes" or _value == "true"
        return False

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def timestamp_iso8601(time_secs=None):
        '''
        Return an ISO UTC timestamp.
        '''
        if time_secs is None:
            if rtc is None:
                return timestamp_iso8601(time.time())
            # (year, month, day, weekday, hours, minutes, seconds, subseconds)
            time_tuple = rtc.datetime()
            return _ISO_FORMAT_STRING.format(
                time_tuple[0],
                time_tuple[1],
                time_tuple[2],
                time_tuple[4],
                time_tuple[5],
                time_tuple[6],
                int(time_tuple[7] / 1000),
            )
        else:
            # (year, month, mday, hour, minute, second, weekday, yearday)
            time_tuple = time.localtime(time_secs)
            return _ISO_FORMAT_STRING.format(
                time_tuple[0],
                time_tuple[1],
                time_tuple[2],
                time_tuple[3],
                time_tuple[4],
                time_tuple[5],
                0,
            )

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def import_configuration(log, filepath):
        '''
        Read configuration from a JSON file.

        :param logger     the logger to capture the result
        :param filepath   the source file path
        '''
#       filepath = 'config.json'
        log.info("importing configuration from file '{}'…".format(filepath))
        with open(filepath) as data_file:
            _config =  json.load(data_file)
            log.info('import complete.')
            return _config

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_formatted_value(value):
        if isinstance(value, float):
            return '{:5.2f}'.format(value)
        else:
            return '{}'.format(value)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def get_formatted_time(label, value):
       if value is None:
           return ''
       elif value > 1000.0:
           return label + ' {:4.3f}s'.format(value/1000.0)
       else:
           return label + ' {:4.3f}ms'.format(value)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def repeat(string, number):
        '''
        Repeat 'string' a given number of times.
        '''
        return (string * (number//len(string) + 1))[:number]

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def ellipsis(string, max_length):
        '''
        Repeat 'string' a given number of times.
        '''
        if len(string) < max_length:
            return string
        else:
            return '{}…'.format(string[:max_length-1])

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def frange(start=0, stop=1, jump=0.1):
        nsteps = int((stop-start)/jump)
        dy = stop-start
        # f(i) goes from start to stop as i goes from 0 to nsteps
        return [start + float(i)*dy/nsteps for i in range(nsteps)]

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def list_methods(cls):
        '''
        Print the methods of the provided class.
        '''
        print(Fore.CYAN + "methods of class: {}".format(type(cls)) + Style.RESET_ALL)
        method_list = [func for func in dir(cls) if callable(getattr(cls, func))]
        for m in method_list:
            print(Fore.CYAN + '    method:\t' + Fore.YELLOW + '{}'.format(m) + Style.RESET_ALL)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def clip(value, min_value, max_value):
        '''
        A replacement for numpy's clip():

            _value = numpy.clip(target_value, _min, _max)
        '''
        return min_value if value <= min_value else max_value if value >= max_value else value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def to_bin(decimal):
        return '{0:08b}'.format(decimal)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def to_bin_v2(x):
        return int(bin(x)[2:])

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def to_decimal(binary):
        b = str(binary)
        binary_len = len(b)
        decimal = 0
        for x in b:
            binary_len = binary_len - 1
            decimal += pow(2,binary_len) * int(x)
        return decimal

#   @staticmethod
#   def clip_alt(n, minimum, maximum):
#       '''
#       Another clip alternative.
#       '''
#       return max(minimum, min(n, maximum))

#EOF
