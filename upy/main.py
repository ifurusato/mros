# main.py -- put your code here!

from pyb import LED
from pyb import Timer
from pyb import I2C
from pyb import Switch
import itertools
import time

#time.sleep(1)           # sleep for 1 second
#time.sleep_ms(500)      # sleep for 500 milliseconds
#time.sleep_us(10)       # sleep for 10 microseconds
#start = time.ticks_ms() # get value of millisecond counter
#delta = time.ticks_diff(time.ticks_ms(), start) # compute time difference

# constants ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

LED_PIN_RED    = 1
LED_PIN_GREEN  = 2
LED_PIN_YELLOW = 3
LED_PIN_BLUE   = 4

SLAVE_ADDRESS  = 0x42
BAUDRATE       = 100000

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
#def callback_method():
#   _count = next(g_counter)
#   if _count % 5 == 0.0:
#   if _count % 50 == 0.0:
#       led_red.toggle()
#   time.sleep_ms(10)
#   led_red.off()
#   led_blue.off()
#   led_green.toggle()

# main ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

led_red    = LED(LED_PIN_RED)    # 1
led_green  = LED(LED_PIN_GREEN)  # 2
led_yellow = LED(LED_PIN_YELLOW) # 3
#led_yellow.intensity(180)
led_blue   = LED(LED_PIN_BLUE)   # 4
led_blue.intensity(64)

#led_blue.on()
#led_blue.off()

# LEDs 3 and 4 support PWM intensity (0-255)
#LED(4).intensity()    # get intensity
#LED(4).intensity(128) # set intensity to half

#g_counter = itertools.count()

#i2c_slave = I2C(1, I2C.SLAVE, addr=SLAVE_ADDRESS, baudrate=BAUDRATE)

#timer_1.counter() # get counter value
#timer_1.freq(0.5) # 0.5 Hz
#timer_1.callback(lambda t: pyb.LED(1).toggle())
#timer_1.callback(lambda t: pyb.LED(1).toggle())
#timer_1 = Timer(1, freq=20)
#timer_1.callback(lambda n: callback_method())

_flash  = True
_switch = Switch()

_data = 42

_active = False

while True:
#   if _active:
#       try:
#           _data = i2c_slave.recv(4)
#       except OSError as exc:
#           if exc.args[0] not in (5, 110):
#               led_yellow.on()
#               # 5 == EIO, occurs when master does a I2C bus scan
#               # 110 == ETIMEDOUT
#               print(exc)
#           else:
#               led_red.on()
#               print('OSError #{}: {}'.format(exc.args[0], exc))
#       except KeyboardInterrupt:
#           break
#       finally:
#           print('rx: {}'.format(_data))

#   else:
#       pass
#       time.sleep_ms(50) 
    if _flash or _switch():
#       _active = not _active
#       if _active:
        led_blue.on()
        time.sleep_ms(20)
#       else:
        led_blue.off()
#       led_yellow.off()
#       led_red.off()
#       time.sleep_ms(333) # debounce
        time.sleep_ms(980) 

#EOF
