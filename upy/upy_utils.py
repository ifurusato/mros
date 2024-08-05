# TinyPICO Utilities
#
# Copyright 2020-2021 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2021-08-26
# modified: 2021-09-13
#

import os
import time
from machine import Pin
from machine import SPI

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#                                 INITIALISE
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

TINY_PICO        = True
ITSYBITSY_RP2040 = False

if TINY_PICO:
    import tinypico as TinyPICO
    from dotstar import DotStar
    import gc

    # configure SPI for controlling the DotStar
    # uses software SPI as the pins used are not hardware SPI pins
    spi = SPI(sck=Pin(TinyPICO.DOTSTAR_CLK), mosi=Pin(TinyPICO.DOTSTAR_DATA), miso=Pin(TinyPICO.SPI_MISO)) # create a DotStar instance
    _dotstar = DotStar(spi, 1, brightness = 0.5)    # just one DotStar, half brightness
    TinyPICO.set_dotstar_power(True)               # turn on the power to the DotStar

if ITSYBITSY_RP2040:
    # NeoPixel power pin 16
    led = Pin(16, Pin.OUT)
    led.value(1)
    # NeoPixel pin 17
    neopx = Neopixel(num_leds=1, state_machine=0, pin=17, mode="RGB")
    neopx.brightness(108)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#                                  CONSTANTS
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

# define constant colours on the Dotstar .........
COLOR_BLACK        = (  0,  0,  0, 10 )
COLOR_RED          = ( 64,  0,  0,  1 )
COLOR_DARK_ORANGE  = (  6,  2,  0,  1 )
COLOR_ORANGE       = ( 32,  8,  0,  1 )
COLOR_YELLOW       = ( 64, 64,  0,  1 )
COLOR_APPLE_GREEN  = ( 32, 84,  0,  1 )
COLOR_GREEN        = (  0, 64,  0,  1 )
COLOR_BLUE         = (  0,  0, 64,  1 )
COLOR_TURQUOISE    = (  0, 10,  7,  1 )
COLOR_SKY_BLUE     = (  4, 12, 50,  1 )
COLOR_CYAN         = (  0, 128, 128, 1 )
COLOR_MAGENTA      = ( 64,  0, 64,  1 )
COLOR_PURPLE       = (  7,  0,  3,  1 )
COLOR_DARK_MAGENTA = ( 10,  0, 10,  1 )
COLOR_WHITE        = ( 64, 64, 64,  1 )

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#                                   FUNCTIONS
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def copy_file(source, target):
    '''
    Copies the source file to the target file, returning True if successful.
    '''
#   try:
#       if os.stat(target)[0] & 0x4000: # is directory
#           target = target.rstrip("/") + "/" + source
#   except OSError as ose:
#       print('OS error: {}'.format(ose))
#       return False
    buf = bytearray(4096)
    buf_mv = memoryview(buf)
    print('-- copying source {} to target{}…'.format(source, target))
    with open(source, "rb") as source, open(target, "wb") as target:
        while (n := source.readinto(buf)) > 0:
            target.write(buf_mv[:n])
    return True

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def file_exists(filename):
    '''
    Returns True if the file exists.
    '''
    try:
        os.stat(filename)
        return True
    except OSError:
        return False

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def rgb_led(color):
    if TINY_PICO:
        _dotstar[0] = color
    if ITSYBITSY_RP2040:
        _neopx.set_pixel(0, color)
        _neopx.show()

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def check_ram():
    '''
    Tests RAM, flash green 3x if okay, flash red 3x otherwise.
    '''
    gc.collect()
    ram = gc.mem_free()
    col = COLOR_GREEN if ram > 4000000 else COLOR_RED
    for i in range (3):
        rgb_led(col)
        time.sleep_ms(50)
        rgb_led(COLOR_BLACK)
        time.sleep_ms(50)
    time.sleep_ms(200)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def rainbow(pos):
    '''
    Input a value 0 to 255 to get a color value.
    The colours are a transition r - g - b - back to r.
    '''
    if pos < 0 or pos > 255:
        return (0, 0, 0)
    elif pos < 85:
        return (int(pos * 3), int(255 - pos * 3), 0)
    elif pos < 170:
        pos -= 85
        return (int(255 - pos * 3), 0, int(pos * 3))
    else:
        pos -= 170
        return (0, int(pos * 3), int(255 - pos * 3))

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def unicorn(count):
    '''
    Unicorns on the RGB LED.
    '''
    for i in range(count):
        for i in range(255):
            r,g,b = rainbow(i)
            rgb_led((r, g, b, 0.5))
            time.sleep(0.002)
    rgb_led(COLOR_BLACK)
    time.sleep_ms(50)

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def ready():
    '''
    Signal readiness.
    '''
    _colors = [COLOR_CYAN]
    for i in range(len(_colors)):
        for j in range(30, 0, -1):
            rgb_led(_colors[i])
            time.sleep_ms(j)
            rgb_led(COLOR_BLACK)
            time.sleep_ms(j)
        rgb_led(COLOR_BLACK)
        time.sleep_ms(60)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#                                   EXECUTE
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

if TINY_PICO:
    check_ram()

unicorn(3)

# signal readiness...
#ready()

#EOF
