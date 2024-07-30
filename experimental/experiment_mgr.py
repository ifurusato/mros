#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   altheim
# created:  2020-09-18
# modified: 2021-10-11
#

import importlib, inspect, traceback
from colorama import init, Fore, Style
init()

import core.globals as globals
from core.logger import Logger, Level
from core.component import Component
from core.event import Event

from experimental.experiment import Experiment

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class ExperimentManager(Component):
    '''
    The ExperimentManager provides a simple framework for providing core
    services to experimental 'sub-projects' as well as hooks for registering
    and triggering them.

    :param config:            the YAML based application configuration
    :param level:             the logging Level
    '''
    def __init__(self, config, suppressed=False, enabled=False, level=Level.INFO):
        if not isinstance(level, Level):
            raise ValueError('wrong type for log level argument: {}'.format(type(level)))
        self._log = Logger("xmgr", level)
        Component.__init__(self, self._log, suppressed, enabled)
        if config is None:
            raise ValueError('no configuration provided.')
        self._registry = {}
        self._config = config['mros'].get('experimental')
        self._directory_name = 'experimental'
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return 'xmgr'

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _instantiate_classes(self):
        if not self.enabled:
            self._log.info('instantiating classes...')
            try:
                for _module_name in self._config:
                    _experiment = self._get_experiment(_module_name)
                    if _experiment:
                        self._log.debug('returned object: {}.'.format(type(_experiment)))
                        _num = len(self._registry) + Event.EXPERIMENT_1.num
                        _event = Event.from_number(_num)
                        self._log.info('created experiment: {} from module {} in slot {:d} for event: {}'.format(
                                _experiment.name, _module_name, _num, _event.name))
                        self._registry[_event] = _experiment
                        self._log.debug('enabling registered experiment {}.'.format(_experiment.name))
                    else:
                        self._log.warning('unable to create instance of module: {}'.format(_module_name))
            except ModuleNotFoundError as mnfe:
                self._log.error('unable to instantiate class: {}\n{}'.format(mnfe, traceback.format_exc()))
            except Exception as e:
                self._log.error('error instantiating experimentsl class: {}\n{}'.format(e, traceback.format_exc()))
        else:
            self._log.warning('experimental classes already instantiated.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _get_experiment(self, module_name):
        self._log.info('get experiment: "{}"'.format(module_name))
        _class_name = self._config[module_name]
        _module = importlib.import_module('{}.{}'.format(self._directory_name, module_name))
        # now scan imports of module for Experiment subclass
        for _name, _obj in inspect.getmembers(_module):
            if inspect.isclass(_obj) and _obj != Experiment and issubclass(_obj, Experiment):
                _type = getattr(_module, _class_name)
                return _type()
        return None

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def register_experiment(self, event, experiment):
        self._registry[event] = experiment
        self._log.info('registered experiment {} to event {}.'.format(experiment.name, event.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def toggle_experiment(self, event):
        _num = event.num - 800
        _experiment = self._registry.get(event)
        if _experiment:
            if _experiment.suppressed:
                self._log.info('release experiment "{}" in slot {:d}'.format(event, _num))
                _experiment.release()
            else:
                self._log.info('suppress experiment "{}" in slot {:d}'.format(event, _num))
                _experiment.suppress()
        else:
            self._log.warning('no registered experiment for event: "{}" (num: {:d})'.format(event, _num))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def print_info(self):
        self._log.info('experiments:')
        for _event in self._registry.keys():
            _experiment = self._registry.get(_event)
            self._log.info('  experiment: ' + Fore.YELLOW + '{}'.format(_experiment.name)
                    + Fore.CYAN + '; registered for event: ' + Fore.YELLOW + '{}'.format(_event.name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def enable(self):
        '''
        Enable this Component.
        '''
        self._instantiate_classes()
        Component.enable(self)
        self._log.info('enabled.')

#EOF
