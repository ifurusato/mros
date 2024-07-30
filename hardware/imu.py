#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020 by Murray Altheim. All rights reserved. This file is part of
# the Robot Operating System project and is released under the "Apache Licence,
# Version 2.0". Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-05-19
# modified: 2024-05-27
#
# _Getch at bottom.
#

import asyncio
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.message_factory import MessageFactory
from core.logger import Logger, Level
from core.event import Event
from core.publisher import Publisher

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class IMU(Publisher):

    CLASS_NAME = 'imu'
    _LISTENER_LOOP_NAME = '__imu_listener_loop'

    '''
    A publisher for events derived from the values of an IMU. It also provides
    general data output from the IMU.

    :param config:            the application configuration
    :param message_bus:       the asynchronous message bus
    :param message_factory:   the factory for creating messages
    :param level:             the log level
    '''
    def __init__(self, config, icm20948, message_bus, message_factory, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._level = level
        self._icm20948 = icm20948
        Publisher.__init__(self, IMU.CLASS_NAME, config, message_bus, message_factory, level=self._level)
        # configuration ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
        _cfg = config['mros'].get('publisher').get('imu')
        _loop_freq_hz         = _cfg.get('loop_freq_hz')
        self._publish_delay_sec = 1.0 / _loop_freq_hz
        self._pitch_threshold = _cfg.get('pitch_threshold')
        self._roll_threshold  = _cfg.get('roll_threshold')
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        if not self._icm20948.is_calibrated:
            self._log.warning('cannot enable IMU: icm20948 has not been calibrated.')
            return
        Publisher.enable(self)
        if self.enabled:
            if self._message_bus.get_task_by_name(IMU._LISTENER_LOOP_NAME):
                self._log.warning('already enabled.')
            else:
                self._log.info('creating task for imu listener loop…')
                self._message_bus.loop.create_task(self._imu_listener_loop(lambda: self.enabled), name=IMU._LISTENER_LOOP_NAME)
                self._log.info('enabled.')
        else:
            self._log.warning('failed to enable publisher.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def icm20948(self):
        '''
        Return the backing IMU.
        '''
        return self._icm20948

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _imu_listener_loop(self, f_is_enabled):
        self._log.info('starting imu listener loop.')
        while f_is_enabled():
            # read heading, pitch and roll from IMU
            _heading, _pitch, _roll = self._icm20948.poll()
#           self._icm20948.poll()
#           _heading = self._icm20948.heading
#           _pitch   = self._icm20948.pitch
#           _roll    = self._icm20948.roll
            _message = None
            if _heading is not None:
                if abs(_roll) > self._roll_threshold:
                    self._log.info('heading: {:.2f}°; pitch: {:.2f}°; '.format(_heading, _pitch) + Style.BRIGHT + ' roll: {:.2f}°'.format(_roll))
                    _message = self.message_factory.create_message(Event.IMU_OVER_ROLL, (_roll))
                elif abs(_pitch) > self._pitch_threshold:
                    self._log.info('imu heading: {:.2f}°;'.format(_heading) + Style.BRIGHT + ' pitch: {:.2f}°;'.format(_pitch) + Style.NORMAL + ' roll: {:.2f}°'.format(_roll))
                    _message = self.message_factory.create_message(Event.IMU_OVER_PITCH, (_pitch))
                else:
#                   self._log.info(Style.DIM + 'imu heading: {:.2f}°; pitch: {:.2f}°; roll: {:.2f}°'.format(_heading, _pitch, _roll))
                    pass
            if _message is not None:
                self._log.info(Style.BRIGHT + 'imu-publishing message:' + Fore.WHITE + Style.NORMAL + ' {}'.format(_message.name)
                        + Fore.CYAN + ' event: {}; '.format(_message.event.name) + Fore.YELLOW + 'timestamp: {}'.format(_message.value))
                await Publisher.publish(self, _message)
            await asyncio.sleep(self._publish_delay_sec)
        self._log.info('imu publish loop complete.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        '''
        Disable this publisher.
        '''
        Publisher.disable(self)
        self._log.info('disabled publisher.')

#EOF
