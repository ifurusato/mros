#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-07-13
# modified: 2024-07-13
#
# Task class at bottom

from colorama import init, Fore, Style
init()

from core.util import Util
from core.logger import Logger, Level
from core.component import Component
from hardware.rotary_encoder import RotaryEncoder, Selector
from hardware.color import Color

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class TaskSelector(Component):
    '''
    Uses a digital encoder to select among an enumerated list of Tasks,
    which are callable functions (with a name and description) added to
    the list by other classes. The number of supported Tasks is limited
    by the number of enumerated colors, currently 16.
    '''
    def __init__(self, rgbmatrix5x5=None, level=Level.INFO):
        self._log = Logger('selector', level)
        Component.__init__(self, self._log, suppressed=False, enabled=True)
        self._rgbmatrix5x5 = rgbmatrix5x5
        self._colors = [
                Color.RED, #           1
                Color.ORANGE, #        2
                Color.YELLOW, #        3
                Color.YELLOW_GREEN, #  4
                Color.GREEN, #         5
                Color.TURQUOISE, #     6
                Color.CYAN, #          7
                Color.SKY_BLUE, #      8
                Color.BLUE, #          9
                Color.BLUE_VIOLET, #  10
                Color.PURPLE, #       11
                Color.MAGENTA, #      12
                Color.FUCHSIA, #      13
                Color.PINK, #         14
                Color.GREY, #         15
                Color.WHITE #         16
            ]
        self._encoder = RotaryEncoder(Level.INFO)
        self._selector = Selector(0, Level.INFO)
        self._max_name_length = 0
        self._tasks = []
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_callback(self, callback):
        '''
        This only works if the interrupt pin is functional on the Rotary Encoder.
        '''
        self._encoder.set_callback(callback)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def add(self, task, name, description):
        '''
        Adds the task to the set of available tasks.
        '''
        if not callable(task): 
            raise ValueError('expected a function, not a {}'.type(task))
        _task_count = len(self._tasks) + 1
        if _task_count > len(self._colors):
            raise Exception('cannot add task: would exceed task count limit of {:d}'.format(len(self._colors)))
     
        _task = Task(_task_count, task, name, description)
        self._max_name_length = max(self._max_name_length, len(name)) 
        self._tasks.append(_task)
        self._selector.set_limit(_task_count)
        self._log.info("added task {:>2}; name: '{}'; description: '{}'".format(_task_count, name, description))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def print_tasks(self):
        '''
        Prints the current list of tasks along with their associated color.
        '''
        if len(self._tasks) == 0:
            self._log.info('no assigned tasks.')
        else:
            _name_len = self._max_name_length
            _format = '  {: <' + str(_name_len + 2) + '}   {}'
            self._log.info(Style.BRIGHT + 'task     color         name {}description'.format(Util.repeat(' ', _name_len)))
            for i in range(len(self._tasks)):
                _color = self._colors[i]
                _task = self._tasks[i]
#               self._log.info('{:d}        {: <8}'.format(i, _color.get_name()) + Fore.YELLOW + '  {: <12}   {}'.format(_task.name, _task.description))
                self._log.info('{:<2}       {: <12}'.format(i, _color.get_name()) + Fore.YELLOW + _format.format(_task.name, _task.description))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def update(self):
        if self.enabled:
            self._value = self._selector.get_value(self._encoder.value())
            self._color = self._colors[self._value]
#           self.set_color(self._color)
            self._encoder.set_rgb(self._color.rgb)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def set_color(self, color):
        '''
        Set the RGB matrix to the provided color, black if the argument is None.
        '''
        if self._rgbmatrix5x5:
            if color:
                self._rgbmatrix5x5.set_all(self._color.red, self._color.green, self._color.blue)
            else:
                self._rgbmatrix5x5.set_all(Color.BLACK.red, Color.BLACK.green, Color.BLACK.blue)
            self._rgbmatrix5x5.show()

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def selection(self):
        '''
        Returns (but does not execute) the currently selected Task object.
        Note that the actual function is its 'task' property.
        '''
        if self.enabled:
            if len(self._tasks) == 0:
                self._log.warning('no assigned tasks.')
                return None
            self.update()
            _task = self._tasks[self._value]
            self._log.info('selection: {: >4}; color: {: >8}; task: {}'.format(self._value, self._color.get_name(), _task.name))
            return _task
        else:
            self._log.warning('disabled: no selection.')
            return None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def color(self):
        '''
        Returns the currently selected color.
        '''
        return self._color

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        if not self.enabled:
            Component.enable(self)
            self._log.info('enabled task selector.')
        elif self.closed:
            self._log.warning('cannot enable task selector: already closed.')
            Component.disable(self)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def disable(self):
        self._tasks.clear()
        Component.disable(self)
        self._encoder.close() 
        self._log.info('disabled task selector.')


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Task(object):
    '''
    A struct containing a Task: its callable function, name and description.
    '''
    def __init__(self, number=-1, task=None, name=None, description=None):
        if not callable(task): 
            raise ValueError('expected a function, not a {}'.type(task))
        self._number = number
        self._task   = task
        self._name   = name
        self._description = description

    @property
    def number(self):
        return self._number

    @property
    def task(self):
        return self._task

    @property
    def name(self):
        return self._name

    @property
    def description(self):
        return self._description

    def __str__(self):
        return '{:d}  {}    {}'.format(self.number, self.name, self.description)

#EOF
