#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2020-04-15
# modified: 2024-06-19

import pprint
from colorama import init, Fore, Style
init()
try:
    import yaml
except ImportError:
    exit("This script requires the pyyaml module\nInstall with: pip3 install --user pyyaml")

from core.logger import Level, Logger

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class ConfigLoader(object):
    '''
    A loader for a YAML configuration file.
    '''
    def __init__(self, level=Level.INFO):
        self._log = Logger('configloader', level)
        self._log.info('ready.')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def configure(self, filename='config.yaml'):
        '''
        Read and return configuration from the specified YAML file.

        Pretty-prints the configuration object if the log level is set to DEBUG.

        :param filename:  the optional name of the YAML file to load. Default: config.yaml
        '''
        self._log.info('reading from YAML configuration file {}…'.format(filename))
        _config = yaml.safe_load(open(filename, 'r'))
        if self._log.level == Level.DEBUG:
            self._log.debug('YAML configuration as read:')
            print(Fore.BLUE)
            pp = pprint.PrettyPrinter(width=80, indent=2)
            pp.pprint(_config)
            print(Style.RESET_ALL)
        self._log.info('configuration read.')
        return _config

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def export(self, config, filename='.config.yaml', comments=None):
        '''
        Export the provided YAML configuration to a file.

        The optional list of comments are written to the beginning of the file.
        '''
        with open(filename, 'w') as fout:
            if comments:
                for comment in comments:
                    fout.write('# {}\n'.format(comment))
                fout.write('#\n\n')
            yaml.dump(config, fout, default_flow_style=False)
        self._log.info('configuration written to {}.'.format(filename))

#EOF
