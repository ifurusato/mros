#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-06-07
# modified: 2024-06-07
#
# Reads the YAML configuration and writes a JSON rendition to the ./upy
# directory, for use by the MicroPython code.
#

from core.logger import Logger, Level
from core.util import Util

_log = Logger('read_config_test', Level.INFO)

try:

    _filepath = 'config.json'
    _read_config = Util.import_configuration(_log, _filepath)
    _log.info('config:\n{}'.format(_read_config))

except Exception as e:
    _log.error('{} encountered writing JSON configuration: {}'.format(type(e), e))

#EOF
