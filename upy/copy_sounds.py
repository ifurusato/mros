#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2024 by Murray Altheim. All rights reserved. This file is part of
# the Robot Operating System project and is released under the "Apache Licence,
# Version 2.0". Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-07-30
# modified: 2024-08-01
#
# Copies all of the WAV files found on the SD card to ./sounds/
#
# Note that this does not delete the existing files in the target directory.
#

import os, uos
from machine import SDCard

import upy_utils

def copy_sound_files():
    '''
    Lists the sound files on the SD card.
    '''
    # note that if the SD card is mounted it can't be used by SoundPlayer
    sd = SDCard(slot=2)  # sck=18, mosi=23, miso=19, cs=5
    vfs = uos.VfsFat(sd)
    uos.mount(vfs, "/sd")
    _copied = 0
    _list = uos.listdir('/sd')
    for _filename in _list:
        if _filename.endswith('.wav'):
            _source_path = '/sd/{}'.format(_filename)
            _target_path = './sounds/{}'.format(_filename)
            _target_exists = upy_utils.file_exists(_target_path)
            if not _target_exists:
                print("copying source file '{}' to target: '{}'".format(_source_path, _target_path))
                _success = upy_utils.copy_file(_source_path, _target_path)
                if _success:
                    print('copied file: {}'.format(_target_path))
                    _copied += 1
                else:
                    print('failed to copy file: {}'.format(_target_path))
            else:
                print('file already exists: /sd/{}; target path: {}'.format(_filename, _target_path))
#           if _copied >= 32:
#               return

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

_flash_info = os.statvfs("/")
statvfs_doc = '''
   [0] f_bsize – file system block size
   [1] f_frsize – fragment size
   [2] f_blocks – size of fs in f_frsize units
   [3] f_bfree – number of free blocks
   [4] f_bavail – number of free blocks for unprivileged users
   [5] f_files – number of inodes
   [6] f_ffree – number of free inodes
   [7] f_favail – number of free inodes for unprivileged users
   [8] f_flag – mount flags
   [9] f_namemax – maximum filename length
'''
_flash_avail = int(_flash_info[0] * _flash_info[3] / 1000)
print('--flash avail: {}k'.format(_flash_avail))

copy_sound_files()

#EOF
