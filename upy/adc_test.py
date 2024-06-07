
import pyb
import time
from colorama import Fore, Style

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

    @property
    def adc(self):
        return self._adc

# ................................................

try:

    print('start.')

    _max = 0.0
    _delay_ms = 80
    _known_limit = 1210 # observed 1218 on MR01

    _a2d = Analog2Digital('Y12', _known_limit)
    _adc = _a2d.adc

    while True:
        _raw   = _adc.read()
        _max = max(_max, _raw)
        _value = _a2d.read()
        print(Fore.BLUE + 'raw: {:.2f}; max: {:.2f}; '.format(_raw, _max) + Fore.WHITE + 'value: {:.2f}'.format(_value))
        time.sleep(_delay_ms / 1000)

    print('done.')

except KeyboardInterrupt:
    print('Ctrl-C caught; exiting...')
except Exception as e:
    print('{} thrown in adc est: {}'.format(type(e), e))
finally:
    print(Style.RESET_ALL)

#EOF
