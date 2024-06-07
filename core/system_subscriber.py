#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-07-19
# modified: 2024-05-19
#

from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.event import Event, Group
from core.subscriber import Subscriber, GarbageCollectedError

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class SystemSubscriber(Subscriber):
    '''
    A subscriber to system events. This is required for shutdown and reacting
    to dire events.

    :param config:       the application configuration
    :param message_bus:  the message bus
    :param level:        the logging level
    '''
    def __init__(self, config, mros, message_bus, level=Level.INFO):
        Subscriber.__init__(self, 'system', config, message_bus=message_bus, suppressed=False, enabled=False, level=level)
        _cfg = config['mros'].get('subscriber').get('system')
        self._mros = mros
        self._message_bus = message_bus
        # exit MROS on dire systems event?
        self._exit_on_dire_event = _cfg.get('exit_on_dire_event')
#       self.add_events(Event.by_groups([Group.SYSTEM, Group.CLOCK]))
        self.add_events([ Group.SYSTEM, Group.CLOCK ])
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def _arbitrate_message(self, message):
        '''
        Pass the message on to the Arbitrator and acknowledge that it has been
        sent (by setting a flag in the message).
        '''
        await self._message_bus.arbitrate(message.payload)
        # increment sent acknowledgement count
        message.acknowledge_sent()
#       self._log.debug('arbitrated payload for event {}; value: {}'.format(message.payload.event.name, message.payload.value))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    async def process_message(self, message):
        '''
        Process the message.

        :param message:  the message to process.
        '''
        if message.gcd:
            raise GarbageCollectedError('cannot process message: message has been garbage collected. [3]')
        _event = message.event
        self._log.debug('pre-processing message {}; '.format(message.name) + Fore.YELLOW + ' event: {}'.format(_event.label))
        if Event.is_system_event(_event):
#           self._log.debug('processing system message {}'.format(message.name))
            self.dispatch_system_event(message.payload)
        elif Event.is_clock_event(_event):
            self.dispatch_system_event(message.payload)
        else:
            self._log.warning('unrecognised event on message {}'.format(message.name) + ''.format(message.event.label))
        await Subscriber.process_message(self, message)
        self._log.debug('post-processing message {}'.format(message.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def dispatch_system_event(self, payload):
        '''
        Process an incoming event's payload.
        '''
        self._log.info('processing payload event {}'.format(payload.event.label))
        if payload.event is Event.TICK:
#           self._log.debug('received clock event message.')
            # TODO check to see if gamepad is still connected
            _gamepad_publisher = self._mros.get_gamepad_publisher()
            if _gamepad_publisher:
                _gamepad = _gamepad_publisher.gamepad
                if _gamepad and _gamepad.has_connection():
                    self._log.debug('gamepad is connected.')
                else:
                    if self._exit_on_dire_event:
                        self._log.error(Fore.MAGENTA + Style.BRIGHT + 'no gamepad found.')
                        self._log.critical('shutting down MROS…')
                        # TODO ideally, brake or halt rather than emergency_stop.
                        self._mros.shutdown()
                    else:
                        self._log.warning(Fore.MAGENTA + Style.BRIGHT + 'no gamepad found.')

        elif payload.event is Event.SHUTDOWN:
            self._log.info('shut down requested.')
            self._mros.shutdown()
        elif payload.event is Event.BATTERY_LOW:
            self._log.critical('battery voltage low!')
            if self._exit_on_dire_event:
                self._log.critical('shutting down MROS…')
                self._mros.shutdown()
            else:
                self._log.critical('WARNING! WARNING! WARNING! battery voltage low! Time to shut down MROS.')
            pass
        elif payload.event is Event.HIGH_TEMPERATURE:
            self._log.critical('high temperature encoutered!')
            if self._exit_on_dire_event:
#               self._message_bus.disable()
#               self._mros.disable()
                pass # TODO
            else:
                self._log.critical('WARNING! WARNING! WARNING! high temperature encountered! Time to go into idle mode.')
         
        elif payload.event is Event.COLLISION_DETECT:
            self._log.critical('collision detection!')
            if self._exit_on_dire_event:
#               self._message_bus.disable()
#               self._mros.disable()
                pass # TODO
            else:
                self._log.critical('WARNING! WARNING! WARNING! collision detection! Stop everything now.')
            pass
        else:
            raise ValueError('unrecognised system event: {}'.format(payload.event.name))

#EOF
