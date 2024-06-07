#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2019-2024 by Murray Altheim. All rights reserved. This file is part
# of the MR01 Robot Operating System (MROS) project, released under the MIT
# License. Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-01-14
# modified: 2024-06-07

# this is a radical simplification of the MROS Logger class, just using print
# statements and not supporting log-to-file, log suppression, etc. As MicroPython
# does not support Enums, a workaround is provided.
#

import math
from colorama import Fore, Style
from core.util import Util

def enum(**enums: int):
    return type('Enum', (), enums)

Level = enum(DEBUG=10, INFO=20, WARN=330, ERROR=40, CRITICAL=50)
# e.g., levels = (Level.ONE, Level.TWO)

## ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#class Level(Enum):
#    DEBUG    = ( logging.DEBUG,    'DEBUG'    ) # 10
#    INFO     = ( logging.INFO,     'INFO'     ) # 20
#    WARN     = ( logging.WARN,     'WARN'     ) # 30
#    ERROR    = ( logging.ERROR,    'ERROR'    ) # 40
#    CRITICAL = ( logging.CRITICAL, 'CRITICAL' ) # 50
#
#    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
#    @staticmethod
#    def from_string(label):
#        if label.upper()   == 'DEBUG':
#            return Level.DEBUG
#        elif label.upper() == 'INFO':
#            return Level.INFO
#        elif label.upper() == 'WARN':
#            return Level.WARN
#        elif label.upper() == 'ERROR':
#            return Level.ERROR
#        elif label.upper() == 'CRITICAL':
#            return Level.CRITICAL
#        else:
#            raise NotImplementedError

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Logger(object):

    __color_debug    = Fore.BLUE   + Style.DIM
    __color_info     = Fore.CYAN   + Style.NORMAL
    __color_notice   = Fore.CYAN   + Style.BRIGHT
    __color_warning  = Fore.YELLOW + Style.NORMAL
    __color_error    = Fore.RED    + Style.NORMAL
    __color_critical = Fore.WHITE  + Style.NORMAL
    __color_reset    = Style.RESET_ALL

    def __init__(self, name, level=Level.INFO):
        '''
        Writes to the console with the provided level.

        :param name:     the name identified with the log output
        :param level:    the log level
        '''
        # configuration ..........................
        self._include_timestamp = True
        self._date_format       = '%Y-%m-%dT%H:%M:%S'
#       self._date_format       = '%Y-%m-%dT%H:%M:%S.%f'
#       self._date_format       = '%H:%M:%S'
        self.__DEBUG_TOKEN = 'DEBUG'
        self.__INFO_TOKEN  = 'INFO '
        self.__WARN_TOKEN  = 'WARN '
        self.__ERROR_TOKEN = 'ERROR'
        self.__FATAL_TOKEN = 'FATAL'
        self._mf           = '{}{} : {}{}'

        # create logger ..........................
        self._name   = name
        self.level = level

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        '''
        Return the name of this Logger.
        '''
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        '''
        Closes down logging, and informs the logging system to perform an
        orderly shutdown by flushing and closing all handlers.

        This is not supported in this implementation, but raises no exception
        when called.
        '''
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def suppress(self):
        '''
        This is not supported in this implementation, but raises no exception
        when called.
        '''
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def release(self):
        '''
        This is not supported in this implementation, but raises no exception
        when called.
        '''
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def level(self):
        '''
        Return the level of this logger.
        '''
        return self._level

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @level.setter
    def level(self, level):
        '''
        Set the level of this logger to the argument.
        '''
        self._level = level

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def is_at_least(self, level):
        '''
        Returns True if the current log level is less than or equals the
        argument. E.g.,

            if self._log.is_at_least(Level.WARN):
                # returns True for WARN or ERROR or CRITICAL
        '''
        return self._level.value >= level.value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def suppressed(self):
        '''
        Return False as suppression is not supported by this logger.
        '''
        return False

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def debug(self, message):
        '''
        Prints a debug message.

        The optional 'end' argument is for special circumstances where a different end-of-line is desired.
        '''
        print(self._mf.format(Logger.__color_debug, self.__DEBUG_TOKEN, message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def info(self, message):
        '''
        Prints an informational message.

        The optional 'end' argument is for special circumstances where a different end-of-line is desired.
        '''
        print(self._mf.format(Logger.__color_info, self.__INFO_TOKEN, message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def notice(self, message):
        '''
        Functionally identical to info() except it prints the message brighter.

        The optional 'end' argument is for special circumstances where a different end-of-line is desired.
        '''
        print(self._mf.format(Logger.__color_notice, self.__INFO_TOKEN, message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def warning(self, message):
        '''
        Prints a warning message.

        The optional 'end' argument is for special circumstances where a different end-of-line is desired.
        '''
        print(self._mf.format(Logger.__color_warning, self.__WARN_TOKEN, message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def error(self, message):
        '''
        Prints an error message.

        The optional 'end' argument is for special circumstances where a different end-of-line is desired.
        '''
        print(self._mf.format(Logger.__color_error, self.__ERROR_TOKEN, Style.NORMAL + message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def critical(self, message):
        '''
        Prints a critical or otherwise application-fatal message.
        '''
        print(self._mf.format(Logger.__color_critical, self.__FATAL_TOKEN, Style.BRIGHT + message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def heading(self, title, message=None, info=None):
        '''
        Print a formatted, titled message to info(), inspired by maven console messaging.

        :param title:    the required title of the heading.
        :param message:  the optional message to display; if None only the title will be displayed.
        :param info:     an optional second message to display right-justified; ignored if None.
        '''
        _H = '┈'
        MAX_WIDTH = 100
        MARGIN = 27
        if title is None or len(title) == 0:
            raise ValueError('no title parameter provided (required)')
        _available_width = MAX_WIDTH - MARGIN
        self.info(self._get_title_bar(title, _available_width))
        if message:
            if info is None:
                info = ''
            _min_msg_width = len(message) + 1 + len(info)
            if _min_msg_width >= _available_width:
                # if total length is greater than available width, just print
                self.info(Fore.WHITE + Style.BRIGHT + '{} {}'.format(message, info))
            else:
                _message_2_right = info.rjust(_available_width - len(message) - 2)
                self.info(Fore.WHITE + Style.BRIGHT + '{} {}'.format(message, _message_2_right))
            # print footer
            self.info(Fore.WHITE + Style.BRIGHT + Util.repeat(_H, _available_width-1))
        # print spacer
        self.info('')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_title_bar(self, message, MAX_WIDTH):
#       _H = '┈'
#       _H = '━'
        _H = '═'
#       _L = '┥ '
        _L = '╡ '
#       _R = ' ┝'
        _R = ' ╞'
        _carrier_width = len(message) + 4
        _hyphen_width = math.floor( ( MAX_WIDTH - _carrier_width ) / 2 )
        if _hyphen_width <= 0:
            return message
        elif len(message) % 2 == 0: # message is even length
            return Fore.WHITE + Style.BRIGHT + Util.repeat(_H, _hyphen_width) + _L + Fore.CYAN + Style.NORMAL\
                    + message + Fore.WHITE + Style.BRIGHT + _R + Util.repeat(_H, _hyphen_width)
        else:
            return Fore.WHITE + Style.BRIGHT + Util.repeat(_H, _hyphen_width) + _L + Fore.CYAN + Style.NORMAL\
                    + message + Fore.WHITE + Style.BRIGHT + _R + Util.repeat(_H, _hyphen_width-1)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class LogStats(object):
    '''
    Provides a simple count for each call to the Logger.
    '''
    def __init__(self):
        self._debug_count    = 0
        self._info_count     = 0
        self._warn_count     = 0
        self._error_count    = 0
        self._critical_count = 0
        pass

    def debug_count(self):
        self._debug_count += 1

    def info_count(self):
        self._info_count += 1

    def warn_count(self):
        self._warn_count += 1

    def error_count(self):
        self._error_count += 1

    def critical_count(self):
        self._critical_count += 1

    @property
    def counts(self):
        return ( self._debug_count, self._info_count, self._warn_count,
                self._error_count, self._critical_count )

#EOF
