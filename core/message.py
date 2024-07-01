#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-03-10
# modified: 2021-03-16
#
# NOTE: to guarantee exactly-once delivery each message must contain a list
# of the identifiers for all current subscribers, with each subscriber
# acknowledgement removing it from that list.
#

import string, uuid, random
from datetime import datetime as dt
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level
from core.stringbuilder import StringBuilder
from core.event import Event
from core.direction import Direction

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Message(object):

    ID_CHARACTERS = string.ascii_uppercase + string.digits

    '''
    IMPORTANT: Don't create one of these directly: use the MessageFactory class.

    :param event:    the Event associated with this Message
    :param value:    the value (or Payload) associated with this Message
    '''
    def __init__(self, event, value):
        if event is None:
            raise ValueError('null event argument.')
        if isinstance(value, Payload):
            print(Fore.GREEN + '🌿 is Payload.' + Style.RESET_ALL)
            self._payload  = value
        else:
            self._payload  = Payload(event, value)
        self._timestamp     = dt.now()
        self._message_id    = uuid.uuid4()
        # generate instance name
        _host_id = "".join(random.choices(Message.ID_CHARACTERS, k=4))
        _instance_name = 'id-{}'.format(_host_id)
        self._instance_name = _instance_name
        self._sent          = 0
        self._expired       = False
        self._gc            = False
        self._processors    = {} # list of processor names who've processed message
        self._subscribers   = {} # list of subscriber names who've acknowledged message

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_subscribers(self, subscribers):
        '''
        Set the list of expected subscribers to this message.
        '''
        for subscriber in subscribers:
            self._subscribers[subscriber] = False

    # instance_name ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def name(self):
        '''
        Return the instance name of the message.
        '''
        return self._instance_name

    # message_id ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def message_id(self):
        return self._message_id

    # timestamp ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def timestamp(self):
        return self._timestamp

    # payload ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def payload(self):
        return self._payload

    # event ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def event(self):
        '''
        A convenience method providing access to the Payload event.
        '''
        return self._payload.event

    # value ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def value(self):
        '''
        A convenience method providing access to the Payload value.
        '''
        return self._payload.value

    @value.setter
    def value(self, value):
        '''
        A convenience method setting the Payload value.
        '''
        self._payload.value = value

    # age ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def age(self):
        _age_ms = (dt.now() - self._timestamp).total_seconds() * 1000.0
        return int(_age_ms)

    # expired ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def expired(self):
        return self._expired

    def expire(self):
        self._expired = True

    # sent ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def sent(self):
        '''
        Returns the number of times this message's payload has been sent to
        the Arbitrator.

        A special case: if a Message returns -1 it effectively means that it
        never expires.
        '''
        return self._sent

    def acknowledge_sent(self):
        '''
        To be called when the message's payload has been sent the Arbitrator.
        '''
        self._sent += 1

    # processed ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def print_procd(self):
        '''
        Returns a pretty-printed list of processors that have processed
        this message, or '[none]' if none.
        '''
        _list = []
        for processor in self._processors:
            if self._processors[processor]:
                _list.append('{} '.format(processor.name))
        return ''.join(_list) if len(_list) > 0 else '[none]'

    @property
    def processed(self):
        return len(self._processors)

    def process(self, processor):
        '''
        Sets the flag that the given processor has finished processing this message.
        '''
        if processor in self._processors:
            raise Exception('message {} ({}) already processed by {}.'.format(self.name, self.event.name, processor.name))
        else:
            self._processors[processor] = True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __hash__(self):
        return hash((self._timestamp, self._message_id, self._instance_name, self._sent, self._expired, self._gc, self._payload))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __eq__(self, other):
        return isinstance(other, Message) and self.__hash__() == other.__hash__()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __str__(self):
        _sb = StringBuilder('Message[', indent=2, delim='\n')
        _sb.append('id={}'.format(id(self)))
        _sb.append('hash={}'.format(hash(self)))
        _sb.append('name={}'.format(self.name))
        _sb.append('timestamp={}'.format(self.timestamp))
        _sb.append('message id={}'.format(self.message_id))
        _sb.append('sent={}'.format(self.sent))
        _sb.append('expired={}'.format(self.expired))
        _sb.append('gcd={}'.format(self.gcd))
        _sb.append('payload={}'.format(self.payload))
        _sb.append(']', indent=0, delim=StringBuilder.NONE)
        return _sb.to_string()

    # garbage collection ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def gcd(self):
        return self._gc

    def gc(self):
        '''
        Garbage collect this message. This sets the 'gc' flag and nullifies
        the event and value properties so no further processing is possible.
        '''
        if self._gc:
            raise Exception('already garbage collected.')
        self._gc = True

    # acknowledged ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    def print_acks(self):
        '''
        Returns a pretty-printed list of subscribers that have acknowledged
        this message, or '[none]' if none.
        '''
        _list = []
        for subscriber in self._subscribers:
            if self._subscribers[subscriber]:
                _list.append('{} '.format(subscriber.name))
        return ''.join(_list) if len(_list) > 0 else '[none]'

#   @property
#   def acknowledgements(self):
#       return self._subscribers

    @property
    def unacknowledged_count(self):
        _count = 0
        for subscriber in self._subscribers:
            if not self._subscribers[subscriber]:
                _count += 1
        return _count

    @property
    def fully_acknowledged(self):
        '''
        Returns True if the message has been acknowledged by all subscribers,
        i.e., no subscriber flags remain set as False, ignoring the garbage
        collector.
        '''
        for subscriber in self._subscribers:
            if not subscriber.is_gc and not self._subscribers[subscriber]:
#               print('message {} not acknowledged by {}'.format(self.name, subscriber.name))
                return False
        return True

    def acknowledged_by(self, subscriber):
        '''
        Returns True if the message has been acknowledged by the specified subscriber.
        '''
        for subscr in self._subscribers:
            if subscr == subscriber and self._subscribers[subscriber]:
                return True
        return False

    def acknowledge(self, subscriber):
        '''
        To be called by each subscriber, acknowledging receipt of the message.
        '''
        if len(self._subscribers) == 0:
            raise Exception('no subscribers set ({}).'.format(self._instance_name))
        if self._subscribers[subscriber]: # if acknowledged already by the subscriber
            if not subscriber.is_gc:
                raise Exception('message {} already acknowledged by subscriber: {}'.format(self.name, subscriber.name))
        else:
            self._subscribers[subscriber] = True

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Payload(object):
    '''
    A Message's payload, containing the Event (with priority) and an optional
    value. The value can be an int, a float, or a tuple containing two ints or
    floats. 

    '''
    def __init__(self, event, value):
        self._event = event
        self._value = value
#   or a tuple containing a Direction and Speed. If the latter this
#   sets the 'is_motor_directive' flag True.
#       self._is_motor_directive = isinstance(value, tuple) \
#               and isinstance(value[0], Direction) and isinstance(value[1], Speed)

    # is motor directive ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

#   @property
#   def is_motor_directive(self):
#       '''
#       Returns True if the arguments to the Payload was a tuple containing
#       a Direction and Speed.
#       '''
#       return self._is_motor_directive

    # priority ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def priority(self):
        return self._event.priority

    # event ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def event(self):
        return self._event

    # direction ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def direction(self):
        return self._value[0]

    # speed ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def speed(self):
        return self._value[1]

    # value ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._value = value

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __hash__(self):
        return hash((self._event, self._value))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __eq__(self, other):
        return isinstance(other, Payload) and self.__hash__() == other.__hash__()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __str__(self):
        _sb = StringBuilder('Payload[', indent=6, delim='\n')
        _sb.append('id={}'.format(id(self)))
        _sb.append('hash={}'.format(hash(self)))
        _sb.append('priority={}'.format(self.priority))
        _sb.append('event={}'.format(self.event))
        _sb.append('value type={}'.format(type(self.value)))
        _sb.append('value={}'.format(self.value))
        _sb.append(']', indent=4, delim=StringBuilder.NONE)
        return _sb.to_string()

    # not equals ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
#  def __ne__(self, other):
#       return not self == other

#EOF
