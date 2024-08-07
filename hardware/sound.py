#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-05-20
# modified: 2024-08-04
#
# DO NOT EDIT: This is an auto-generated file.
#

import json
from enum import Enum
from colorama import init, Fore, Style
init()

from core.logger import Logger, Level

# JSON serialisation ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
from json import JSONEncoder
def _default(self, obj):
    return getattr(obj.__class__, "to_json", _default.default)(obj)
_default.default = JSONEncoder().default
JSONEncoder.default = _default

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class Sound(Enum):
    __order__ = " SILENCE CHATTER_1 CHATTER_2 CHATTER_3 CHATTER_4 CHATTER_5 CHIRP_1 CHIRP_2 TELEMETRY KLAXON BIPPIT BUZZ BZAT DWERP EARPIT EEPIT_AR_WOT HZAH ITIZ IZIT PEW_PEW_PEW PING PIZZLE POKE SONIC_BAT SPIN_DOWN TWEAK VIBRATE TWIDDLE_POP CHATTER TINY_GLITCH SKID_FZZT ZZT "

    #                idx   name            mnemonic     dur(s)  filename                description
    SILENCE       = (  1, 'silence',      'SILENCE',      0.0, 'silence.wav',      'silence')
    CHATTER_1     = (  2, 'chatter-1',    'CHATTER_1',    1.0, 'chatter-1.wav',    'chatter 1')
    CHATTER_2     = (  3, 'chatter-2',    'CHATTER_2',    1.0, 'chatter-2.wav',    'chatter 2')
    CHATTER_3     = (  4, 'chatter-3',    'CHATTER_3',    2.0, 'chatter-3.wav',    'chatter 3')
    CHATTER_4     = (  5, 'chatter-4',    'CHATTER_4',    1.0, 'chatter-4.wav',    'chatter 4')
    CHATTER_5     = (  6, 'chatter-5',    'CHATTER_5',    1.0, 'chatter-5.wav',    'chatter 5')
    CHIRP_1       = (  7, 'chirp-1',      'CHIRP_1',      2.0, 'chirp-1.wav',      'chirp 1 6')
    CHIRP_2       = (  8, 'chirp-2',      'CHIRP_2',      2.0, 'chirp-2.wav',      'chirp 2 7')
    TELEMETRY     = (  9, 'telemetry',    'TELEMETRY',    1.0, 'telemetry.wav',    'telemetry 8')
    KLAXON        = ( 10, 'klaxon',       'KLAXON',       2.0, 'klaxon.wav',       'major problem')
    BIPPIT        = ( 11, 'bippit',       'BIPPIT',       1.0, 'bippit.wav',       'bippit 11')
    BUZZ          = ( 12, 'buzz',         'BUZZ',         1.0, 'buzz.wav',         'buzz 12')
    BZAT          = ( 13, 'bzat',         'BZAT',         1.0, 'bzat.wav',         'bzat 13')
    DWERP         = ( 14, 'dwerp',        'DWERP',        1.0, 'dwerp.wav',        'dwerp 14')
    EARPIT        = ( 15, 'earpit',       'EARPIT',       1.0, 'earpit.wav',       'ear pit 15')
    EEPIT_AR_WOT  = ( 16, 'eepit-ar-wot', 'EEPIT_AR_WOT', 1.0, 'eepit-ar-wot.wav', 'eepit ar wot? 16')
    HZAH          = ( 17, 'hzah',         'HZAH',         1.0, 'hzah.wav',         'hzah 17')
    ITIZ          = ( 18, 'itiz',         'ITIZ',         1.0, 'itiz.wav',         'it iz. 18')
    IZIT          = ( 19, 'izit',         'IZIT',         1.0, 'izit.wav',         'iz it? 19')
    PEW_PEW_PEW   = ( 20, 'pew-pew-pew',  'PEW_PEW_PEW',  1.0, 'pew-pew-pew.wav',  'pew pew pew 20')
    PING          = ( 21, 'ping',         'PING',         3.0, 'ping.wav',         'sonar ping 21')
    PIZZLE        = ( 22, 'pizzle',       'PIZZLE',       1.0, 'pizzle.wav',       'pizzle 22')
    POKE          = ( 23, 'poke',         'POKE',         1.0, 'poke.wav',         'poke 23')
    SONIC_BAT     = ( 24, 'sonic-bat',    'SONIC_BAT',    1.0, 'sonic-bat.wav',    'sonic bat beep 24')
    SPIN_DOWN     = ( 25, 'spin-down',    'SPIN_DOWN',    1.0, 'spin-down.wav',    'spin down 25')
    TWEAK         = ( 26, 'tweak',        'TWEAK',        1.0, 'tweak.wav',        'tweak 26')
    VIBRATE       = ( 27, 'vibrate',      'VIBRATE',      1.0, 'vibrate.wav',      'error of some kind')
    TWIDDLE_POP   = ( 28, 'twiddle-pop',  'TWIDDLE_POP',  1.0, 'twiddle-pop.wav',  'twiddle pop 28')
    CHATTER       = ( 29, 'chatter',      'CHATTER',      1.0, 'chatter.wav',      'chatter 29')
    TINY_GLITCH   = ( 30, 'tiny-glitch',  'TINY_GLITCH',  1.0, 'tiny-glitch.wav',  'tiny glitch 30')
    SKID_FZZT     = ( 31, 'skid-fzzt',    'SKID_FZZT',    1.0, 'skid-fzzt.wav',    'skid fzzt 31')
    ZZT           = ( 32, 'zzt',          'ZZT',          1.0, 'zzt.wav',          'zzt 32')

    # ignore the first param since it's already set by __new__
    def __init__(self, num, name, mnemonic, duration, filename, description):
        self._index = num
        self._name = name
        self._mnemonic = mnemonic
        self._duration = duration
        self._filename = filename
        self._description = description

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def index(self):
        return self._index

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def name(self):
        return self._name

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def mnemonic(self):
        return self._mnemonic

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def duration(self):
        return self._duration

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def filename(self):
        return self._filename

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def description(self):
        return self._description

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def from_index(value):
        if isinstance(value, Sound):
            _index = value.index
        elif isinstance(value, int):
            _index = value
        else:
            raise Exception('expected Sound or int argument, not: {}'.format(type(value)))
        for s in Sound:
            if _index == s.index:
                return s
        raise NotImplementedError

    # JSON serialisation ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @staticmethod
    def export():
        _records = []
        for _record in Sound:
            _records.append(_record)
        with open('sounds.json', 'w', encoding='utf-8') as f:
            json.dump(_records, f, ensure_ascii=False, indent=4)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def to_json(self):
        _dict = {}
        _dict['index']       = self._index
        _dict['name']        = self._name
        _dict['mnemonic']    = self._mnemonic
        _dict['duration']    = self._duration
        _dict['filename']    = self._filename
        _dict['description'] = self._description
        return _dict

#EOF
