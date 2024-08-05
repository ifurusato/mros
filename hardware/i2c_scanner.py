#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   altheim
# created:  2020-02-14
# modified: 2021-08-11
#
#  Scans the I²C bus, returning a list of devices.
#
# see: https://www.raspberrypi.org/forums/viewtopic.php?t=114401
# see: https://raspberrypi.stackexchange.com/questions/62612/is-there-anyway-to-scan-i2c-using-pure-python-libraries:q
#
# If you're getting a "Permission denied" message due to smbus, add the pi user to the i2c group using:
#
#  % sudo adduser pi i2c
#
# then reboot.
#
# DeviceNotFound class at bottom.
#

import errno
from colorama import init, Fore, Style
init()

from core.logger import Level, Logger

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class I2CScanner(object):
    '''
    Scans the I²C bus, returning a list of devices.
    '''
    def __init__(self, config, bus_number=1, level=Level.INFO):
        super().__init__()
        if not isinstance(bus_number, int):
            raise ValueError('expected bus number as an int.')
        elif not isinstance(level, Level):
            raise ValueError('expected log level as a Level enum.')
        self._log = Logger('i2cscan', level)
        self._config = config
        self._int_list = []
        self._hex_list = []
        self._bus      = None
        try:
            self._log.info('initialising…')
            from smbus2 import SMBus
            _bus_number = bus_number  # 1 indicates /dev/i2c-1
            self._bus = SMBus(_bus_number)
            self._log.info('ready.')
        except ImportError:
            self._log.warning('import error, unable to initialise: this script requires smbus2. Scan will return an empty result.')
        except Exception as e:
            self._log.warning('{} while initialising I²C bus: scan will return an empty result.'.format(e))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_hex_addresses(self):
        '''
        Returns a hexadecimal version of the list.
        '''
        self._scan_addresses()
        return self._hex_list

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_int_addresses(self):
        '''
        Returns an integer version of the list.
        '''
        self._scan_addresses()
        return self._int_list

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def has_address(self, addresses):
        '''
        Performs the address scan (if necessary) and returns true if a device
        is available at any of the specified int addresses, the argument a list
        of strings.
        '''
        self._scan_addresses()
        for address in addresses:
            if address in self._int_list:
                return True
        return False

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def normalise(self, address):
        '''
        Uppercases the numerical part of the hex string.
        '''
        return '0x{}'.format(address[2:].upper())

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def has_hex_address(self, addresses):
        '''
        Performs the address scan (if necessary) and returns true if a device
        is available at any of the specified hexadecimal addresses, the argument
        a list of strings.
        '''
        self._scan_addresses()
        for address in addresses:
            address = self.normalise(address)
            if address in self._hex_list:
                return True
        return False

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def _scan_addresses(self):
        '''
        Scans the bus and returns the available device addresses. After being
        called and populating the int and hex lists, this closes the connection
        to smbus.
        '''
        if len(self._int_list) == 0:
            self._log.info('scanning I²C address bus…')
            device_count = 0
            if self._bus is not None:
                for address in range(3, 128):
                    try:
                        self._bus.write_byte(address, 0)
                        _hex_address = hex(address)
                        self._log.debug('found I²C device at 0x{:02X} (hex: {})'.format(address, _hex_address))
                        self._int_list.append(address)
                        self._hex_list.append('0x{:02X}'.format(address))
                        device_count = device_count + 1
                    except IOError as e:
                        if e.errno != errno.EREMOTEIO:
                            self._log.debug('{0} on address {1}'.format(e, hex(address)))
#                           self._log.warning('{0} on address {1}'.format(e, hex(address)))
                    except Exception as e: # exception if read_byte fails
                        self._log.error('{0} error on address {1}'.format(e, hex(address)))
                self._bus.close()
                self._bus = None
                self._log.info("found {:d} I²C device(s).".format(device_count))
            else:
                self._log.info("found no devices (no smbus available).")
        return self._int_list

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def print_device_list(self):
        self._addrDict = dict(list(map(lambda x, y:(x,y), self.get_int_addresses(), self.get_hex_addresses())))
        for _address in self.get_int_addresses():
            _device_name = self.get_device_for_address(_address)
            self._log.info('  I²C address 0x{:02X}: '.format(_address) + Fore.YELLOW + '{}'.format(_device_name))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def get_device_for_address(self, address):
        '''
        Returns the lookup device name from the device registry found in
        the YAML configuration.
        '''
        _device = self._config['devices'].get(address)
        return 'Unknown' if _device is None else _device

    # end class ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class DeviceNotFound(Exception):
    '''
    Thrown when an expected device cannot be found.
    '''
    pass

# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
def main():

    level = Level.INFO
    log = Logger('main', level)
    log.info('scanning for I²C devices…')
    scanner = I2CScanner(level=Level.INFO)

    _addresses = scanner.get_int_addresses()
    log.info('available I²C device(s):')
    if len(_addresses) == 0:
        log.warning('no devices found.')
        return
    else:
        for n in range(len(_addresses)):
            address = _addresses[n]
            log.info('device: {0} ({1})'.format(address, hex(address)))

    for i in range(len(_addresses)):
        print(Fore.CYAN + '-- address: {}'.format(_addresses[i]) + Style.RESET_ALL)

    print('')

if __name__== "__main__":
    main()

#EOF
