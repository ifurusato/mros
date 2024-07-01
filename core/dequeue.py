#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-09-25
# modified: 2021-10-15
#

import copy
from copy import deepcopy
from queue import Queue, LifoQueue, Empty, Full
from colorama import init, Fore, Style
init(autoreset=True)

from core.stringbuilder import StringBuilder

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DeQueue(object):

    FIFO = QUEUE = 0 # use FIFO queue
    LIFO = STACK = 1 # use LIFO queue (like a stack)

    '''
    A "double-ended" FIFO or LIFO queue based on the CPython Queue class.

    :param maxsize:        optional maximum length of the queue.
    :param backing_queue:  optional backing queue
    :param mode:           uses a FIFO (default) queue or LIFO if thus specified
    '''
    def __init__(self, maxsize=-1, backing_queue=None, mode=FIFO):
        self._maxsize = maxsize
        self._mode = mode
        if mode == DeQueue.LIFO: # implemented as LIFO stack
            self._queue = LifoQueue(maxsize)
        else:                    # implemented as FIFO queue
            self._queue = Queue(maxsize)
        if backing_queue:
            self._backing_queue = self._queue.queue
            self._queue.queue   = backing_queue
        else:
            self._backing_queue = self._queue.queue

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def queue(self):
        '''
        Return the backing queue.
        '''
        return self._backing_queue

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def iterator(self):
        '''
        Return an iterator over the entries in the queue.
        '''
        return self._queue

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def size(self):
        '''
        Return the size of the queue.
        '''
        return self._queue.qsize()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def clear(self):
        '''
        Clears the contents of the queue.
        '''
        with self._queue.mutex:
            self._queue.queue.clear()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def empty(self):
        '''
        Returns True if the queue is empty.
        '''
        return self.size == 0

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def full(self):
        '''
        Returns True if the queue has reached its maximum length.
        '''
        return self._queue.full()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def peek(self):
        '''
        Return the next item on the queue without getting it.
        This functions correctly for either a FIFO queue or a LIFO stack.
        '''
        if self.empty:
            raise Empty()
        if self._mode == DeQueue.FIFO:
            return self._backing_queue[0]
        elif self._mode == DeQueue.LIFO: # stack
            return self._backing_queue[self.size-1]

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def poll(self):
        '''
        Return the next item on the queue.
        '''
        if self.empty:
            raise Empty()
        if self._mode == DeQueue.LIFO:
            raise TypeError('poll() unsuppported in LIFO mode (stack), use pop() or the more general get().')
        return self._queue.get(self._queue.qsize() - 1)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def put_as_set(self, item):
        '''
        This puts the item onto the queue only if it doesn't already exist,
        creating a Set behaviour. This doesn't change the order of the existing
        items in the queue.
        '''
        if item not in self._queue:
            self.put(item)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def push(self, item):
        '''
        An alias for put(item). 
        '''
        self.put(item)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def put(self, item):
        '''
        Put the item onto the queue.

        This has an additional feature over the default in that will ignore
        null arguments rather than raise an exception.
        '''
        if self.full:
            raise Full()
        if item:
            self._queue.put(item)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def pop(self):
        '''
        An alias for get().
        '''
        if self._mode == DeQueue.FIFO:
            raise TypeError('pop() unsuppported in FIFO mode (queue), use poll() or the more general get().')
        return self.get()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get(self):
        '''
        Get the next item from the queue.
        '''
        if self.empty:
            raise Empty()
        return self._queue.get()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __eq__(self, other):
        return isinstance(other, DeQueue) \
                and self.__hash__() == other.__hash__() \
                and self._queue == other.queue

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __hash__(self):
        return hash((self._maxsize, self._mode, self._queue.qsize()))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __str__(self):
        _sb = StringBuilder('  DeQueue[', indent=6, delim='\n')
        _sb.append('id={}'.format(id(self)))
        _sb.append('hash={}'.format(hash(self)))
        _sb.append('size={}'.format(self.size))
        _sb.append('maxsize={}'.format(self._maxsize))
        _sb.append('mode={}'.format(self._mode))
        _sb.append('items:')
        for _stmt in self._queue.queue:
            _sb.append(_stmt, indent=8)
        _sb.append(']', indent=6, delim=StringBuilder.NONE)
        return _sb.to_string()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def __deepcopy__(self, memo):
        return DeQueue(maxsize=self._maxsize, backing_queue=copy.deepcopy(self._backing_queue), mode=self._mode)

    # unimplemented...
#   def replace(self, item):
#   def putget(self, item):
#   def queueify(x):
#   def nlargest(n, iterable):
#   def nsmallest(n, iterable):
#   def merge(*iterables):
#   def nsmallest(n, iterable, key=None):
#   def nlargest(n, iterable, key=None):

## ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#class Empty(Exception):
#    '''
#    An exception thrown when an action is taken upon an empty queue.
#    '''
#    pass
#
## ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#class Full(Exception):
#    '''
#    An exception thrown when the queue would overflow upon fulfilling an action.
#    '''
#    pass

#EOF
