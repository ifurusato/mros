#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-08-01
# modified: 2024-04-09
#
# MR01 Robot Operating System Daemon (mrosd). This also uses the mrosd.service.
#
# see: https://dpbl.wordpress.com/2017/02/12/a-tutorial-on-python-daemon/
#
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

import os, signal, sys, time, threading, traceback, itertools

try:
    import daemon
    from daemon import pidfile
except Exception:
    sys.exit("This script requires the python-daemon module.\nInstall with: pip3 install --user python-daemon")

from datetime import datetime
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger
from core.config_loader import ConfigLoader
from mros import MROS
from hardware.toggle import Toggle

PIDFILE = '/home/pi/mros/.mrosd.pid'

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def shutdown(signum, frame):  # signum and frame are mandatory
    sys.exit(0)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
class MrosDaemon():
    '''
    Monitors a toggle switch connected to a GPIO pin.

    This state is used to enable or disable the MROS. This replaces rather
    than reuses the Status class (as a reliable simplification).
    '''
    def __init__(self, level):
        self._level = level
        self._log = Logger("mrosd", self._level)
        self._log.info('initialising mrosd...')

        # read YAML configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _loader = ConfigLoader(self._level)
        _filename = 'config.yaml'
        self._config = _loader.configure(_filename)

        _mrosd_config = self._config['mrosd']
        print(Fore.MAGENTA + "_mrosd_config: {}".format(_mrosd_config) + Style.RESET_ALL)
        _toggle_pin = _mrosd_config.get('toggle_pin')
        print(Fore.CYAN + "_toggle_pin: {}".format(_toggle_pin) + Style.RESET_ALL)
        self._toggle = Toggle(_toggle_pin, Level.WARN)
        _application = _mrosd_config.get('application') # 'mros'
        self._log.info('mrosd application set to: {}'.format(_application))

        self._counter   = itertools.count()
        self._old_state = False
        self._mros      = None
        _rosd_mask = os.umask(0)
        os.umask(_rosd_mask)
        self._log.info('mask: {}'.format(_rosd_mask))
        self._log.info('uid:  {}'.format(os.getuid()))
        self._log.info('gid:  {}'.format(os.getgid()))
        self._log.info('cwd:  {}'.format(os.getcwd()))
        self._log.info('pid file: {}'.format(PIDFILE))
        self._log.info('mrosd ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_timestamp(self):
        return datetime.utcfromtimestamp(datetime.utcnow().timestamp()).isoformat()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def read_state(self):
        '''
        Reads the state of the toggle switch then returns the value. This only
        calls enable() or disable() if the value has changed since last reading.
        '''
        if next(self._counter) % 30 == 0: # call every second, log every 30 seconds
            self._log.info('mros daemon waiting...')
        self._state = self._toggle.state
        if self._state is not self._old_state:
            if self._state:
                self._log.info('enabling from state: {}'.format(self._state))
                self.enable()
            else:
                self._log.info('disabling from state: {}'.format(self._state))
                self.disable()
            self._old_state = self._state

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        self._enable_mros()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        self._disable_mros()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _enable_mros(self):
        self._log.info('mros state enabled at: {}'.format(self._get_timestamp()))
        if self._mros is None:
            self._log.info('starting mros thread...')
            _level = Level.INFO
            self._mros = MROS(_level) # (mutex=self._log.mutex)
            self._mros.start()
            time.sleep(1.0)
            self._log.info('mros started.')
        else:
            self._log.info('enabling mros arbitrator...')
#           _arbitrator = self._mros.get_arbitrator()
#           _arbitrator.set_suppressed(False)
            self._log.info('mros arbitrator enabled.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _disable_mros(self):
        self._log.info('mros state disabled at: {}'.format(self._get_timestamp()))
        if self._mros is not None:
            self._log.info('suppressing mros arbitrator... ')
#           _arbitrator = self._mros.get_arbitrator()
#           _arbitrator.set_suppressed(True)
            self._log.info('mros arbitrator suppressed.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def close(self):
        self._log.info('closing mrosd...')
        if self._mros is not None:
            self._log.info('closing mros...')
            self._mros.close()
            self._log.info('mros closed.')
        else:
            self._log.warning('mros thread was null.')
        self._log.info('mrosd closed.')


# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

def main():

    _daemon = None

    try:
        _daemon = MrosDaemon(Level.INFO)
        while True:
            _daemon.read_state()
            time.sleep(1.0)

    except Exception:
        print('error starting mros daemon: {}'.format(traceback.format_exc()))
    finally:
        if _daemon:
            try:
                _daemon.close()
            except Exception:
                print('error closing mros daemon.')
        print('mrosd complete.')

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

with daemon.DaemonContext(
    stdout=sys.stdout,
    stderr=sys.stderr,
#   chroot_directory=None,
    working_directory='/home/pi/mros',
    umask=0o002,
    pidfile=pidfile.TimeoutPIDLockFile(PIDFILE), ) as context:
#   signal_map={ signal.SIGTERM: shutdown, signal.SIGTSTP: shutdown }) as context:
    main()

#EOF
