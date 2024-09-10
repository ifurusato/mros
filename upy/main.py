#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2024 by Murray Altheim. All rights reserved. This file is part of
# the Robot Operating System project and is released under the "Apache Licence,
# Version 2.0". Please see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2024-03-30
# modified: 2024-08-10
#
# This is the script for the port-side TinyPICO, labeled "8", /dev/ttyUSB0.
#
# Receives input on five pins treated as a binary number, then triggers
# one of 32 sounds from the I2S Audio Shield.
#

import os, uos
from machine import SoftSPI, Pin
from machine import Timer
from machine import SDCard
import tinypico as TinyPICO
from dotstar import DotStar
import time, gc

import upy_utils
import itertools
from sound import Sound
from sound_player import SoundPlayer

COPY_SOUND_FILES = False

# ESP32-Now ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

# set up pins on the ESP32 representing bits of an int

PIN_0    = 22 # 1 on IOE
PIN_1    = 33 # 2 on IOE
PIN_2    =  4 # 3 on IOE
PIN_3    = 32 # 4 on IOE
PIN_4    = 14 # 5 on IOE
PIN_5    = 21 # 6 on IOE
PIN_6    = 15 # 7 on IOE


pin0 = Pin(PIN_0, Pin.IN, Pin.PULL_UP)
pin1 = Pin(PIN_1, Pin.IN, Pin.PULL_UP)
pin2 = Pin(PIN_2, Pin.IN, Pin.PULL_UP)
pin3 = Pin(PIN_3, Pin.IN, Pin.PULL_UP)
pin4 = Pin(PIN_4, Pin.IN, Pin.PULL_UP)
pin5 = Pin(PIN_5, Pin.IN, Pin.PULL_UP)
pin6 = Pin(PIN_6, Pin.IN, Pin.PULL_UP)

last_index = None

# SD card support ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

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
        if _copied >= 32:
            return

# TinyPICO main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

# define constant colours on the Dotstar ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
COLOR_BLACK        = (  0,  0,  0, 10 )
COLOR_RED          = ( 64,  0,  0,  1 )  # 0
COLOR_DARK_ORANGE  = (  6,  2,  0,  1 )  #
COLOR_ORANGE       = ( 32,  8,  0,  1 )  # 1
COLOR_YELLOW       = ( 64, 64,  0,  1 )  # 2
COLOR_APPLE_GREEN  = ( 32, 84,  0,  1 )  #
COLOR_GREEN        = (  0, 64,  0,  1 )  # 3
COLOR_BLUE         = (  0,  0, 64,  1 )  # 4
COLOR_TURQUOISE    = (  0, 10,  7,  1 )  #
COLOR_SKY_BLUE     = (  4, 12, 50,  1 )  #
COLOR_CYAN         = (  0, 128, 128, 1 ) # 5
COLOR_MAGENTA      = ( 64,  0, 64,  1 )  # 6
COLOR_PURPLE       = (  7,  0,  3,  1 )  # 7
COLOR_DARK_MAGENTA = ( 10,  0, 10,  1 )  #
COLOR_WHITE        = ( 64, 64, 64,  1 )  #

COLORS = [ COLOR_BLACK, COLOR_MAGENTA, COLOR_ORANGE, COLOR_CYAN, COLOR_YELLOW, COLOR_GREEN, COLOR_BLUE, COLOR_RED, COLOR_WHITE ]

# configure SPI for controlling the DotStar
# use software SPI as the pins used are not hardware SPI pins
spi = SoftSPI(sck=Pin(TinyPICO.DOTSTAR_CLK), mosi=Pin(TinyPICO.DOTSTAR_DATA), miso=Pin(TinyPICO.SPI_MISO)) # create a DotStar instance
dotstar = DotStar(spi, 1, brightness = 0.5)    # just one DotStar, half brightness
TinyPICO.set_dotstar_power(True)               # turn on the power to the DotStar

# TinyPICO functions ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

# test RAM, flash green 3x if okay, flash red 3x otherwise
def check_ram():
    gc.collect()
    ram = gc.mem_free()
    col = COLOR_GREEN if ram > 4000000 else COLOR_RED
    for i in range (3):
        dotstar[0] = col
        time.sleep_ms(50)
        dotstar[0] = COLOR_BLACK
        time.sleep_ms(50)
    time.sleep_ms(250)

def get_selected_index():
    '''
    Converts the seven input pin values as a binary-coded decimal (BCD),
    returning an int between 0 and 127.
    '''
    return int('{}{}{}{}{}{}{}'.format(pin6.value(), pin5.value(), pin4.value(), pin3.value(), pin2.value(), pin1.value(), pin0.value()), 2)

def poll():
    '''
    Gets the BCD index, playing the sound if the index has changed since the last poll. 
    Sending '00000' between sounds is interpreted as a reset.
    '''
    global last_index
    _index = get_selected_index()
    if last_index != None and _index != last_index and _index > 0:
        play_sound(_index)
    last_index = _index
    if next(_counter) % 12 == 0:
        dotstar[0] = COLOR_SKY_BLUE
        time.sleep_ms(4)
        dotstar[0] = COLOR_BLACK

def play_sound(index):
    _sound = sounds[index]
    _filename = _sound.filename
    _source_path = './sounds/{}'.format(_filename)
    print("-- playing sound #{:d} from source: {}".format(index, _source_path))
    player = SoundPlayer(_source_path)
    player.play()

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

# check the RAM
check_ram()

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
print('flash avail: {}k'.format(_flash_avail))

# sound files ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

if COPY_SOUND_FILES:
    copy_sound_files()
 
sounds = Sound.load_sounds()
#for sound in sounds:
#    print('sound: {}'.format(sound))

# signal readiness...
for j in range(0, 3):
    dotstar[0] = COLOR_APPLE_GREEN
    time.sleep_ms(333)
    dotstar[0] = COLOR_BLACK
    time.sleep_ms(333)

_counter = itertools.count()

# start timer with the set frequency
_timer = Timer(1)
_freq_hz=5
_timer.init(freq=_freq_hz, mode=Timer.PERIODIC, callback=lambda n: poll())

#EOF
