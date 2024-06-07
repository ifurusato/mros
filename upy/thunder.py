
import sys
import pyb
import time
import thunderborg3
from colorama import Fore, Style

KNOWN_LIMIT = 1210 # observed 1218 on MR01

# ..............................................................................
class Analog2Digital(object):

    def __init__(self, pin, limit):
        self._adc = pyb.ADC(pin)
        self._limit = limit
        print('ready.')

    def read(self):
        '''
        Based on the known limit, returns a value betweeen 0.0 and 1.0.
        '''
        return self._adc.read() / self._limit

# ..............................................................................

try:

    LED_TEST   = False
    MOTOR_TEST = False

    _a2d = Analog2Digital('Y12', KNOWN_LIMIT)

    _tb = thunderborg3.ThunderBorg()
    _tb.Init()

    _voltage = _tb.GetBatteryReading()
    print(Fore.GREEN + 'battery: {:.2f}v'.format(_voltage))
    time.sleep(3)

    if LED_TEST:
        # LED test .................................................................
        _tb.SetLedShowBattery(False)
        for _ in range(7):
            _tb.SetLed1(0.1, 0.0, 0.1) # dull purple
            time.sleep(0.2)
            _tb.SetLed1(0.7, 0.0, 0.7) # brighter purple
            time.sleep(0.2)
        _tb.SetLeds(1.0, 0.0, 0.0) # -> red
        time.sleep(2)
        _tb.SetLeds(0.0, 0.0, 0.0) # black

    _delay_ms = 80
    _count = 0

    while True:
        _count += 1
        _value = _a2d.read()
        _motor_power = _value
        _tb.SetMotors(_motor_power) 
#       _tb.SetMotor1(_motor_power) 
#       _tb.SetMotor2(_motor_power) 
        if _count % 10 == 0: 
            print(Fore.BLUE + 'value: {:.2f}; '.format(_value) + Fore.WHITE + 'motor power: {:.2f}'.format(_value, _motor_power))
        time.sleep(_delay_ms / 1000)

    # motor test ...............................................................

    if MOTOR_TEST:
        _delay_s = 0.2
        _max_power = 5
        print('ThunderBorg Port Motor Test…')
        for _power in range(0, _max_power):
            _motor_power = _power / 10
            print('motor power: {:.2f}'.format(_motor_power))
            _tb.SetMotor1(_motor_power) 
            time.sleep(_delay_s)
        for _power in range(_max_power, 0, -1):
            _motor_power = _power / 10
            print('motor power: {:.2f}'.format(_motor_power))
            _tb.SetMotor1(_motor_power) 
            time.sleep(_delay_s)
        _tb.SetMotor1(0.0)
        print('ThunderBorg Starboard Motor Test…')
        for _power in range(0, _max_power):
            _motor_power = _power / 10
            print('motor power: {:.2f}'.format(_motor_power))
            _tb.SetMotor2(_motor_power) 
            time.sleep(_delay_s)
        for _power in range(_max_power, 0, -1):
            _motor_power = _power / 10
            print('motor power: {:.2f}'.format(_motor_power))
            _tb.SetMotor2(_motor_power) 
            time.sleep(_delay_s)
        _tb.SetMotor2(0.0)

    print('ready.')

except Exception as e:
    print('{} thrown in thunderborg test: {}'.format(type(e), e))
    sys.print_exception(e)
finally:
    if _tb:
        _tb.SetLedShowBattery(True)
        _tb.SetMotor1(0.0)
        _tb.SetMotor2(0.0)
    print('finally.')

#EOF
