#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2019-2021 by Murray Altheim. All rights reserved. This file is part
# of the MR01 Robot Operating System (MROS) project, released under the MIT
# License. Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-01-14
# modified: 2024-05-19
#

import os, logging, math, traceback, threading
from logging.handlers import RotatingFileHandler
from datetime import datetime as dt
from enum import Enum
from colorama import init, Fore, Style
init()

import core.globals as globals
globals.init()

from core.util import Util
from core.ansi_filtering_file_handler import AnsiFilteringRotatingFileHandler

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Level(Enum):
    DEBUG    = ( logging.DEBUG,    'DEBUG'    ) # 10
    INFO     = ( logging.INFO,     'INFO'     ) # 20
    WARN     = ( logging.WARN,     'WARN'     ) # 30
    ERROR    = ( logging.ERROR,    'ERROR'    ) # 40
    CRITICAL = ( logging.CRITICAL, 'CRITICAL' ) # 50

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __new__(cls, *args, **kwds):
        obj = object.__new__(cls)
        obj._value_ = args[0]
        return obj

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    # ignore the first param since it's already set by __new__
    def __init__(self, num, label):
        self._label = label

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def from_string(label):
        if label.upper()   == 'DEBUG':
            return Level.DEBUG
        elif label.upper() == 'INFO':
            return Level.INFO
        elif label.upper() == 'WARN':
            return Level.WARN
        elif label.upper() == 'ERROR':
            return Level.ERROR
        elif label.upper() == 'CRITICAL':
            return Level.CRITICAL
        else:
            raise NotImplementedError

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Logger(object):

    __suppress       = False
    __color_debug    = Fore.BLUE   + Style.DIM
    __color_info     = Fore.CYAN   + Style.NORMAL
    __color_notice   = Fore.CYAN   + Style.BRIGHT
    __color_warning  = Fore.YELLOW + Style.NORMAL
    __color_error    = Fore.RED    + Style.NORMAL
    __color_critical = Fore.WHITE  + Style.NORMAL
    __color_reset    = Style.RESET_ALL

    def __init__(self, name, log_to_console=True, log_to_file=False, level=Level.INFO):
        '''
        Writes to a named log with the provided level, defaulting to a
        console (stream) handler unless 'log_to_file' is True, in which
        case only write to file, not to the console.

        :param name:           the name identified with the log output
        :param log_to_console:  if True will log to console (default True)
        :param log_to_file:    if True will subsequentially log to file, for all loggers (default False)
        :param level:          the log level
        '''
        # configuration preliminaries ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        if log_to_file:
            _log_to_file = log_to_file
            globals.put('log-to-file', True)
        elif globals and globals.has('log-to-file'):
            _log_to_file = globals.get('log-to-file')
        else:
            _log_to_file = False

        # get or create log statistics object
        _log_stats = globals.get('log-stats')
        if not _log_stats:
            _log_stats = LogStats()
            globals.put('log-stats', _log_stats)
        self._log_stats = _log_stats

        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _strip_ansi_codes       = True # used only with file output, to strip ANSI characters from log data
        self._include_timestamp = True
        self._date_format       = '%Y-%m-%dT%H:%M:%S'
#       self._date_format       = '%Y-%m-%dT%H:%M:%S.%f'
#       self._date_format       = '%H:%M:%S'
        # i18n?
        self.__DEBUG_TOKEN = 'DEBUG'
        self.__INFO_TOKEN  = 'INFO '
        self.__WARN_TOKEN  = 'WARN '
        self.__ERROR_TOKEN = 'ERROR'
        self.__FATAL_TOKEN = 'FATAL'
        self._mf           = '{}{} : {}{}'
        _1st_col_width     = 14

        # create logger  ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        self.__mutex = threading.Lock()
        self.__log   = logging.getLogger(name)
        self.__log.propagate = False
        self._name   = name
        self._fh     = None # optional file handler
        self._sh     = None # optional stream handler
        if not self.__log.handlers:
            if log_to_console: # log to console ┈┈┈┈┈┈┈┈┈┈┈┈
                self._sh = logging.StreamHandler()
                if self._include_timestamp:
                    self._sh.setFormatter(logging.Formatter(Fore.BLUE + Style.DIM + '%(asctime)s.%(msecs)3fZ\t:' \
                            + Fore.RESET + ' %(name)s ' + ( ' '*(_1st_col_width-len(name)) ) + ' : %(message)s', datefmt=self._date_format))
#                   self._sh.setFormatter(logging.Formatter('%(asctime)s.%(msecs)06f  %(name)s ' + ( ' '*(_1st_col_width-len(name)) ) + ' : %(message)s', datefmt=self._date_format))
                else:
                    self._sh.setFormatter(logging.Formatter('%(name)s ' + ( ' '*(_1st_col_width-len(name)) ) + ' : %(message)s'))
                self.__log.addHandler(self._sh)
            if _log_to_file: # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
                # if ./log/ directory doesn't exist, create it
                if not os.path.exists('./log'):
                    try:
                        os.makedirs('./log')
                    except OSError as e:
                        raise Exception('could not create ./log directory: {}'.format(e))
                _ts = dt.utcfromtimestamp(dt.utcnow().timestamp()).isoformat().replace(':','_').replace('-','_').replace('.','_')
                _filename = './log/mros-{}.csv'.format(_ts)
                self.info("logging to file: {}".format(_filename))
                # do we already have a file handler?
                if globals.has('log-file-handler'): # use existing file handler
                    self._fh = globals.get('log-file-handler')
                elif _strip_ansi_codes: # using new ANSI filtering file handler
                    self._fh = AnsiFilteringRotatingFileHandler(filename=_filename, mode='w', maxBytes=262144, backupCount=10)
                    globals.put('log-file-handler', self._fh)
                else: # using new rotating file handler
                    self._fh = RotatingFileHandler(filename=_filename, mode='w', maxBytes=262144, backupCount=10)
                    globals.put('log-file-handler', self._fh)
#               self._fh.setLevel(level.value)
                if self._include_timestamp:
                    self._fh.setFormatter(logging.Formatter('%(asctime)s.%(msecs)03dZ\t|%(name)s|%(message)s', datefmt=self._date_format))
                else:
                    self._fh.setFormatter(logging.Formatter('%(name)s|%(message)s'))
                self.__log.addHandler(self._fh)

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
        orderly shutdown by flushing and closing all handlers. This should
        be called at application exit and no further use of the logging
        system should be made after this call.
        '''
#       self.suppress()
        logging.shutdown()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def suppress(self):
        '''
        Suppresses all log messages except critical errors and log-to-file
        messages. This is global across all Loggers.
        '''
        type(self).__suppress = True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def release(self):
        '''
        Releases (un-suppresses) all log messages except critical errors
        and log-to-file messages. This is global across all Loggers.
        '''
        type(self).__suppress = False

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
        self.__log.setLevel(self._level.value)
        if self._fh:
            self._fh.setLevel(level.value)
        if self._sh:
            self._sh.setLevel(level.value)

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
    def stats(self):
        '''
        Return the global log statistics.
        '''
        return self._log_stats

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def suppressed(self):
        '''
        Return True if this logger has been suppressed.
        '''
        return type(self).__suppress

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def debug(self, message):
        '''
        Prints a debug message.

        The optional 'end' argument is for special circumstances where a different end-of-line is desired.
        '''
        if not self.suppressed:
            self._log_stats.debug_count()
            with self.__mutex:
                self.__log.debug(self._mf.format(Logger.__color_debug, self.__DEBUG_TOKEN, message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def info(self, message):
        '''
        Prints an informational message.

        The optional 'end' argument is for special circumstances where a different end-of-line is desired.
        '''
        if not self.suppressed:
            self._log_stats.info_count()
            with self.__mutex:
                self.__log.info(self._mf.format(Logger.__color_info, self.__INFO_TOKEN, message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def notice(self, message):
        '''
        Functionally identical to info() except it prints the message brighter.

        The optional 'end' argument is for special circumstances where a different end-of-line is desired.
        '''
        if not self.suppressed:
            self._log_stats.info_count()
            with self.__mutex:
                self.__log.info(self._mf.format(Logger.__color_notice, self.__INFO_TOKEN, message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def warning(self, message):
        '''
        Prints a warning message.

        The optional 'end' argument is for special circumstances where a different end-of-line is desired.
        '''
        if not self.suppressed:
            self._log_stats.warn_count()
            with self.__mutex:
                self.__log.warning(self._mf.format(Logger.__color_warning, self.__WARN_TOKEN, message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def error(self, message):
        '''
        Prints an error message.

        The optional 'end' argument is for special circumstances where a different end-of-line is desired.
        '''
        if not self.suppressed:
            self._log_stats.error_count()
            with self.__mutex:
                self.__log.error(self._mf.format(Logger.__color_error, self.__ERROR_TOKEN, Style.NORMAL + message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def critical(self, message):
        '''
        Prints a critical or otherwise application-fatal message.
        '''
        with self.__mutex:
            self._log_stats.critical_count()
            self.__log.critical(self._mf.format(Logger.__color_critical, self.__FATAL_TOKEN, Style.BRIGHT + message, Logger.__color_reset))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def file(self, message):
        '''
        This is just info() but without any formatting.
        '''
        with self.__mutex:
            self._log_stats.info_count()
            self.__log.info(message)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def heading(self, title, message=None, info=None):
        '''
        Print a formatted, titled message to info(), inspired by maven console messaging.

        :param title:    the required title of the heading.
        :param message:  the optional message to display; if None only the title will be displayed.
        :param info:     an optional second message to display right-justified; ignored if None.
        '''
        if not self.suppressed:
            self._log_stats.info_count()
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
