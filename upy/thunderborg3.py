#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# MicroPython port of CPython
#
# Copyright 2020-2024 by Murray Altheim. All rights reserved. This file is part
# of the Robot Operating System project, released under the MIT License. Please
# see the LICENSE file included as part of this package.
#
# author:   Murray Altheim
# created:  2019-12-23
# modified: 2024-05-18
#
# This module is designed to communicate with the ThunderBorg
#
# Use by creating an instance of the class, call the Init function,
# then command as desired, e.g.
#
#     import ThunderBorg
#     TB = ThunderBorg.ThunderBorg()
#     TB.Init()
#     # User code here, use TB to control the board
#
# For explanations of the functions available call the Help function, e.g.
#
#     import ThunderBorg
#     TB = ThunderBorg.ThunderBorg()
#     TB.Help()
#     See the website at www.piborg.org/thunderborg for more details
#
# ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

import machine
from machine import I2C
import io, types, time

# Constant values
I2C_SLAVE                   = 0x0703
PWM_MAX                     = 255
I2C_MAX_LEN                 = 6
VOLTAGE_PIN_MAX             = 36.3  # Maximum voltage from the analog voltage monitoring pin
VOLTAGE_PIN_CORRECTION      = 0.0   # Correction value for the analog voltage monitoring pin
BATTERY_MIN_DEFAULT         = 7.0   # Default minimum battery monitoring voltage
BATTERY_MAX_DEFAULT         = 35.0  # Default maximum battery monitoring voltage

I2C_ID_THUNDERBORG          = 0x15

COMMAND_SET_LED1            = 1     # Set the colour of the ThunderBorg LED
COMMAND_GET_LED1            = 2     # Get the colour of the ThunderBorg LED
COMMAND_SET_LED2            = 3     # Set the colour of the ThunderBorg Lid LED
COMMAND_GET_LED2            = 4     # Get the colour of the ThunderBorg Lid LED
COMMAND_SET_LEDS            = 5     # Set the colour of both the LEDs
COMMAND_SET_LED_BATT_MON    = 6     # Set the colour of both LEDs to show the current battery level
COMMAND_GET_LED_BATT_MON    = 7     # Get the state of showing the current battery level via the LEDs
COMMAND_SET_A_FWD           = 8     # Set motor A PWM rate in a forwards direction
COMMAND_SET_A_REV           = 9     # Set motor A PWM rate in a reverse direction
COMMAND_GET_A               = 10    # Get motor A direction and PWM rate
COMMAND_SET_B_FWD           = 11    # Set motor B PWM rate in a forwards direction
COMMAND_SET_B_REV           = 12    # Set motor B PWM rate in a reverse direction
COMMAND_GET_B               = 13    # Get motor B direction and PWM rate
COMMAND_ALL_OFF             = 14    # Switch everything off
COMMAND_GET_DRIVE_A_FAULT   = 15    # Get the drive fault flag for motor A, indicates faults such as short-circuits and under voltage
COMMAND_GET_DRIVE_B_FAULT   = 16    # Get the drive fault flag for motor B, indicates faults such as short-circuits and under voltage
COMMAND_SET_ALL_FWD         = 17    # Set all motors PWM rate in a forwards direction
COMMAND_SET_ALL_REV         = 18    # Set all motors PWM rate in a reverse direction
COMMAND_SET_FAILSAFE        = 19    # Set the failsafe flag, turns the motors off if communication is interrupted
COMMAND_GET_FAILSAFE        = 20    # Get the failsafe flag
COMMAND_GET_BATT_VOLT       = 21    # Get the battery voltage reading
COMMAND_SET_BATT_LIMITS     = 22    # Set the battery monitoring limits
COMMAND_GET_BATT_LIMITS     = 23    # Get the battery monitoring limits
COMMAND_WRITE_EXTERNAL_LED  = 24    # Write a 32bit pattern out to SK9822 / APA102C
COMMAND_GET_ID              = 0x99  # Get the board identifier
COMMAND_SET_I2C_ADD         = 0xAA  # Set a new I2C address

COMMAND_VALUE_FWD           = 1     # I2C value representing forward
COMMAND_VALUE_REV           = 2     # I2C value representing reverse

COMMAND_VALUE_ON            = 1     # I2C value representing on
COMMAND_VALUE_OFF           = 0     # I2C value representing off

COMMAND_ANALOG_MAX          = 0x3FF # Maximum value for analog readings

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class ThunderBorg:
    '''
    This module is designed to communicate with the ThunderBorg

    busNumber        I2C bus on which the ThunderBorg is attached (Rev 1 is bus 0, Rev 2 is bus 1)
    bus              the smbus object used to talk to the I2C bus
    i2cAddress       The I2C address of the ThunderBorg chip to control
    foundChip        True if the ThunderBorg chip can be seen, False otherwise
    printFunction    Function reference to call when printing text, if None "print" is used
    '''
    
    # Shared values used by this class              
    busNumber               = 1                   # Check here for Rev 1 vs Rev 2 and select the correct bus
    i2cAddress              = I2C_ID_THUNDERBORG  # I2C address, override for a different address
    foundChip               = False
    printFunction           = None
    i2cWrite                = None
    i2cRead                 = None
    
    def __init__(self, i2c_address=I2C_ID_THUNDERBORG):
        super().__init__()
#       self._log = Logger('thunderborg', level)
#       self._log.info('ready.')
        self._i2c_address = i2c_address
        self.Print('initialising thunderboard at 0x{:02x}.'.format(self._i2c_address))
        self.Print('ready.')
    
    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def RawWrite(self, command, data):
        """
        RawWrite(command, data)

        Sends a raw command on the I2C bus to the ThunderBorg
        Command codes can be found at the top of ThunderBorg.py, data is a list of 0 or more byte values

        Under most circumstances you should use the appropriate function instead of RawWrite
        """
        rawOutput = [command]
        rawOutput.extend(data)
        rawOutput = bytes(rawOutput)
#       self.i2cWrite.write(rawOutput)
        self._i2c.writeto(self._i2c_address, rawOutput)
    

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def RawRead(self, command, length, retryCount = 3):
        """
        RawRead(command, length, [retryCount])

        Reads data back from the ThunderBorg after sending a GET command
        Command codes can be found at the top of ThunderBorg.py, length is the number of bytes to read back

        The function checks that the first byte read back matches the requested command
        If it does not it will retry the request until retryCount is exhausted (default is 3 times)

        Under most circumstances you should use the appropriate function instead of RawRead
        """
        while retryCount > 0:
            self.RawWrite(command, [])
#           rawReply = self.i2cRead.read(length)
            rawReply = self._i2c.readfrom(self._i2c_address, length) 
            reply = []
            for singleByte in rawReply:
                reply.append(singleByte)
            if command == reply[0]:
                break
            else:
                retryCount -= 1
        if retryCount > 0:
            return reply
        else:
            raise IOError('I2C read for command %d failed'.format(command))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def Init(self, bus_number=1):
        """
        Prepare the I2C driver for talking to the ThunderBorg. The default I2C bus is 1.
        """
        self.Print('Loading ThunderBorg on bus {:d}, address 0x{:02X}'.format(self.busNumber, self.i2cAddress))
        
        print('creating i2c connection...')
        self._i2c = machine.I2C(bus_number)
        
        print('scanning i2c bus...')
        devices = self._i2c.scan()
        if len(devices) == 0:
            print("no i2c devices found!")
            self.foundChip = False
        else:
            print('{} i2c devices found:'.format(len(devices)))
            for device in devices:
                print("decimal address: {}; hex address: {}".format(device, hex(device)))

        # Check for ThunderBorg
        try:
            i2cRecv = self.RawRead(COMMAND_GET_ID, I2C_MAX_LEN)
            if len(i2cRecv) == I2C_MAX_LEN:
                if i2cRecv[1] == I2C_ID_THUNDERBORG:
                    self.foundChip = True
                    self.Print('Found ThunderBorg at 0x{:02X}'.format(self.i2cAddress))
                else:
                    self.foundChip = False
                    self.Print('Found a device at 0x{:02X}, but it is not a ThunderBorg (ID 0x{:02X} instead of 0x{:02X})'.format(self.i2cAddress, i2cRecv[1], I2C_ID_THUNDERBORG))
            else:
                self.foundChip = False
                self.Print('Missing ThunderBorg at 0x{:02X}'.format(self.i2cAddress))
        except KeyboardInterrupt:
            raise
        except:
            self.foundChip = False
            self.Print('Missing ThunderBorg at 0x{:02X}'.format(self.i2cAddress))

        # See if we are missing chips
        if not self.foundChip:
            self.Print('ThunderBorg was not found!')
            self.Print('Are you sure your ThunderBorg is properly attached, the correct address is used, and the I2C drivers are running?')
            self.bus = None
        else:
            self.Print('ThunderBorg loaded on bus %d'.format(self.busNumber))

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    @property
    def I2cAddress(self):
        '''
        For diagnostics only; not to be used directly.
        '''
        return self.i2cAddress

    # Motors ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def SetMotors(self, power):
        """
        SetMotors(power)

        Sets the drive level for all motors, from +1 to -1.
        e.g.
        SetMotors(0)     -> all motors are stopped
        SetMotors(0.75)  -> all motors are moving forward at 75% power
        SetMotors(-0.5)  -> all motors are moving reverse at 50% power
        SetMotors(1)     -> all motors are moving forward at 100% power
        """
        if power < 0:
            # Reverse
            command = COMMAND_SET_ALL_REV
            pwm = -int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX
        else:
            # Forward / stopped
            command = COMMAND_SET_ALL_FWD
            pwm = int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX

        try:
            self.RawWrite(command, [pwm])
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending all motors drive level!')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def MotorsOff(self):
        """
        MotorsOff()

        Sets all motors to stopped, useful when ending a program
        """
        try:
            self.RawWrite(COMMAND_ALL_OFF, [0])
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending motors off command!')

    # Motor 1 ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def SetMotor1(self, power):
        """
        SetMotor1(power)

        Sets the drive level for motor 1, from +1 to -1.
        e.g.
        SetMotor1(0)     -> motor 1 is stopped
        SetMotor1(0.75)  -> motor 1 moving forward at 75% power
        SetMotor1(-0.5)  -> motor 1 moving reverse at 50% power
        SetMotor1(1)     -> motor 1 moving forward at 100% power
        """
#       print('SetMotor1() power: {}'.format(power))
        if power < 0:
            # Reverse
            command = COMMAND_SET_A_REV
            pwm = -int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX
        else:
            # Forward / stopped
            command = COMMAND_SET_A_FWD
            pwm = int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX

#       self.Print('SetMotor1({}).'.format(power))
#       self._log.info('SetMotor1({}).'.format(power))
        try:
            self.RawWrite(command, [pwm])
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending motor 1 drive level!')


    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def SetMotor1Off(self):
        '''
        Bespoke method (not part of ThunderBorg API code).
        '''
        try:
            command = COMMAND_SET_A_FWD
            self.RawWrite(command, [0])
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed zeroing motor 1 drive level!')


    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def GetMotor1(self):
        """
        power = GetMotor1()

        Gets the drive level for motor 1, from +1 to -1.
        e.g.
        0     -> motor 1 is stopped
        0.75  -> motor 1 moving forward at 75% power
        -0.5  -> motor 1 moving reverse at 50% power
        1     -> motor 1 moving forward at 100% power
        """
        try:
            i2cRecv = self.RawRead(COMMAND_GET_A, I2C_MAX_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading motor 1 drive level!')
#           self._log.debug('failed reading motor 1 drive level.')
            return None

        power = float(i2cRecv[2]) / float(PWM_MAX)

        if i2cRecv[1] == COMMAND_VALUE_FWD:
            return power
        elif i2cRecv[1] == COMMAND_VALUE_REV:
            return -power
        else:
            return

    # Motor 2 ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def SetMotor2(self, power):
        """
        SetMotor2(power)

        Sets the drive level for motor 2, from +1 to -1.
        e.g.
        SetMotor2(0)     -> motor 2 is stopped
        SetMotor2(0.75)  -> motor 2 moving forward at 75% power
        SetMotor2(-0.5)  -> motor 2 moving reverse at 50% power
        SetMotor2(1)     -> motor 2 moving forward at 100% power
        """
#       print('SetMotor2() power: {}'.format(power))
        if power < 0:
            # Reverse
            command = COMMAND_SET_B_REV
            pwm = -int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX
        else:
            # Forward / stopped
            command = COMMAND_SET_B_FWD
            pwm = int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX

#       self._log.info(Fore.GREEN + 'SetMotor2({}).'.format(power))
        try:
            self.RawWrite(command, [pwm])
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending motor 2 drive level!')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def SetMotor2Off(self):
        '''
        Bespoke method (not part of ThunderBorg API code).
        '''
        try:
            command = COMMAND_SET_B_FWD
            self.RawWrite(command, [0])
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed zeroing motor 2 drive level!')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def GetMotor2(self):
        """
        power = GetMotor2()

        Gets the drive level for motor 2, from +1 to -1.
        e.g.
        0     -> motor 2 is stopped
        0.75  -> motor 2 moving forward at 75% power
        -0.5  -> motor 2 moving reverse at 50% power
        1     -> motor 2 moving forward at 100% power
        """
        try:
            i2cRecv = self.RawRead(COMMAND_GET_B, I2C_MAX_LEN)
        except KeyboardInterrupt:
            raise
        except:
#           self.Print('Failed reading motor 2 drive level!')
            self._log.debug('failed reading motor 2 drive level.')
            return None

        power = float(i2cRecv[2]) / float(PWM_MAX)

        if i2cRecv[1] == COMMAND_VALUE_FWD:
            return power
        elif i2cRecv[1] == COMMAND_VALUE_REV:
            return -power
        else:
            return

    # LEDs ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def SetLed1(self, r, g, b):
        """
        SetLed1(r, g, b)

        Sets the current colour of the ThunderBorg LED. r, g, b may each be between 0 and 1
        e.g.
        SetLed1(0, 0, 0)       -> ThunderBorg LED off
        SetLed1(1, 1, 1)       -> ThunderBorg LED full white
        SetLed1(1.0, 0.5, 0.0) -> ThunderBorg LED bright orange
        SetLed1(0.2, 0.0, 0.2) -> ThunderBorg LED dull purple
        """
        levelR = max(0, min(PWM_MAX, int(r * PWM_MAX)))
        levelG = max(0, min(PWM_MAX, int(g * PWM_MAX)))
        levelB = max(0, min(PWM_MAX, int(b * PWM_MAX)))

        try:
            self.RawWrite(COMMAND_SET_LED1, [levelR, levelG, levelB])
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending colour for the ThunderBorg LED!')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def GetLed1(self):
        """
        r, g, b = GetLed1()

        Gets the current colour of the ThunderBorg LED. r, g, b may each be between 0 and 1
        e.g.
        0, 0, 0       -> ThunderBorg LED off
        1, 1, 1       -> ThunderBorg LED full white
        1.0, 0.5, 0.0 -> ThunderBorg LED bright orange
        0.2, 0.0, 0.2 -> ThunderBorg LED dull purple
        """
        try:
            i2cRecv = self.RawRead(COMMAND_GET_LED1, I2C_MAX_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading ThunderBorg LED colour!')
            return

        r = i2cRecv[1] / float(PWM_MAX)
        g = i2cRecv[2] / float(PWM_MAX)
        b = i2cRecv[3] / float(PWM_MAX)
        return r, g, b

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def SetLed2(self, r, g, b):
        """
        SetLed2(r, g, b)

        Sets the current colour of the ThunderBorg Lid LED. r, g, b may each be between 0 and 1
        e.g.
        SetLed2(0, 0, 0)       -> ThunderBorg Lid LED off
        SetLed2(1, 1, 1)       -> ThunderBorg Lid LED full white
        SetLed2(1.0, 0.5, 0.0) -> ThunderBorg Lid LED bright orange
        SetLed2(0.2, 0.0, 0.2) -> ThunderBorg Lid LED dull purple
        """
        levelR = max(0, min(PWM_MAX, int(r * PWM_MAX)))
        levelG = max(0, min(PWM_MAX, int(g * PWM_MAX)))
        levelB = max(0, min(PWM_MAX, int(b * PWM_MAX)))

        try:
            self.RawWrite(COMMAND_SET_LED2, [levelR, levelG, levelB])
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending colour for the ThunderBorg Lid LED!')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def GetLed2(self):
        """
        r, g, b = GetLed2()

        Gets the current colour of the ThunderBorg Lid LED. r, g, b may each be between 0 and 1
        e.g.
        0, 0, 0       -> ThunderBorg Lid LED off
        1, 1, 1       -> ThunderBorg Lid LED full white
        1.0, 0.5, 0.0 -> ThunderBorg Lid LED bright orange
        0.2, 0.0, 0.2 -> ThunderBorg Lid LED dull purple
        """
        try:
            i2cRecv = self.RawRead(COMMAND_GET_LED2, I2C_MAX_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading ThunderBorg Lid LED colour!')
            return

        r = i2cRecv[1] / float(PWM_MAX)
        g = i2cRecv[2] / float(PWM_MAX)
        b = i2cRecv[3] / float(PWM_MAX)
        return r, g, b

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def SetLeds(self, r, g, b):
        """
        SetLeds(r, g, b)

        Sets the current colour of both LEDs. r, g, b may each be between 0 and 1
        e.g.
        SetLeds(0, 0, 0)       -> Both LEDs off
        SetLeds(1, 1, 1)       -> Both LEDs full white
        SetLeds(1.0, 0.5, 0.0) -> Both LEDs bright orange
        SetLeds(0.2, 0.0, 0.2) -> Both LEDs dull purple
        """
        levelR = max(0, min(PWM_MAX, int(r * PWM_MAX)))
        levelG = max(0, min(PWM_MAX, int(g * PWM_MAX)))
        levelB = max(0, min(PWM_MAX, int(b * PWM_MAX)))

        try:
            self.RawWrite(COMMAND_SET_LEDS, [levelR, levelG, levelB])
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending colour for both LEDs!')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def SetLedShowBattery(self, state):
        """
        SetLedShowBattery(state)

        Sets the system to enable or disable the LEDs showing the current battery level
        If enabled the LED colours will be ignored and will use the current battery reading instead
        This sweeps from fully green for maximum voltage (35 V) to fully red for minimum voltage (7 V)
        """
        if state:
            level = COMMAND_VALUE_ON
        else:
            level = COMMAND_VALUE_OFF

        try:
            self.RawWrite(COMMAND_SET_LED_BATT_MON, [level])
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending LED battery monitoring state!')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def GetLedShowBattery(self):
        """
        state = GetLedShowBattery()

        Gets if the system is using the LEDs to show the current battery level, true for enabled, false for disabled
        If enabled the LED colours will be ignored and will use the current battery reading instead
        This sweeps from fully green for maximum voltage (35 V) to fully red for minimum voltage (7 V)
        """
        try:
            i2cRecv = self.RawRead(COMMAND_GET_LED_BATT_MON, I2C_MAX_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading LED battery monitoring state!')
            return

        if i2cRecv[1] == COMMAND_VALUE_OFF:
            return False
        else:
            return True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def Print(self, message):
        """
        Print(message)

        Wrapper used by the ThunderBorg instance to print(messages, will call printFunction if set, print otherwise)
        """
#       self._log.info(message)
        if self.printFunction == None:
            print(message)
        else:
            self.printFunction(message)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def NoPrint(self, message):
        """
        NoPrint(message)

        Does nothing, intended for disabling diagnostic printout by using:
        TB = ThunderBorg.ThunderBorg()
        TB.printFunction = TB.NoPrint
        """
        pass

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def SetCommsFailsafe(self, state):
        """
        SetCommsFailsafe(state)

        Sets the system to enable or disable the communications failsafe
        The failsafe will turn the motors off unless it is commanded at least once every 1/4 of a second
        Set to True to enable this failsafe, set to False to disable this failsafe
        The failsafe is disabled at power on
        """
        if state:
            level = COMMAND_VALUE_ON
        else:
            level = COMMAND_VALUE_OFF

        try:
            self.RawWrite(COMMAND_SET_FAILSAFE, [level])
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending communications failsafe state!')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def GetCommsFailsafe(self):
        """
        state = GetCommsFailsafe()

        Read the current system state of the communications failsafe, True for enabled, False for disabled
        The failsafe will turn the motors off unless it is commanded at least once every 1/4 of a second
        """
        try:
            i2cRecv = self.RawRead(COMMAND_GET_FAILSAFE, I2C_MAX_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading communications failsafe state!')
            return

        if i2cRecv[1] == COMMAND_VALUE_OFF:
            return False
        else:
            return True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def GetDriveFault1(self):
        """
        state = GetDriveFault1()

        Reads the motor drive fault state for motor #1, False for no problems, True for a fault has been detected
        Faults may indicate power problems, such as under-voltage (not enough power), and may be cleared by setting a lower drive power
        If a fault is persistent, it repeatably occurs when trying to control the board, this may indicate a wiring problem such as:
            * The supply is not powerful enough for the motors
                The board has a bare minimum requirement of 6V to operate correctly
                A recommended minimum supply of 7.2V should be sufficient for smaller motors
            * The + and - connections for motor #1 are connected to each other
            * Either + or - is connected to ground (GND, also known as 0V or earth)
            * Either + or - is connected to the power supply (V+, directly to the battery or power pack)
            * One of the motors may be damaged
        Faults will self-clear, they do not need to be reset, however some faults require both motors to be moving at less than 100% to clear
        The easiest way to check is to put both motors at a low power setting which is high enough for them to rotate easily, such as 30%
        Note that the fault state may be true at power up, this is normal and should clear when both motors have been driven
        For more details check the website at www.piborg.org/thunderborg and double check the wiring instructions
        """
        try:
            i2cRecv = self.RawRead(COMMAND_GET_DRIVE_A_FAULT, I2C_MAX_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading the drive fault state for motor #1!')
            return

        if i2cRecv[1] == COMMAND_VALUE_OFF:
            return False
        else:
            return True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def GetDriveFault2(self):
        """
        state = GetDriveFault2()

        Reads the motor drive fault state for motor #2, False for no problems, True for a fault has been detected
        Faults may indicate power problems, such as under-voltage (not enough power), and may be cleared by setting a lower drive power
        If a fault is persistent, it repeatably occurs when trying to control the board, this may indicate a wiring problem such as:
            * The supply is not powerful enough for the motors
                The board has a bare minimum requirement of 6V to operate correctly
                A recommended minimum supply of 7.2V should be sufficient for smaller motors
            * The + and - connections for motor #2 are connected to each other
            * Either + or - is connected to ground (GND, also known as 0V or earth)
            * Either + or - is connected to the power supply (V+, directly to the battery or power pack)
            * One of the motors may be damaged
        Faults will self-clear, they do not need to be reset, however some faults require both motors to be moving at less than 100% to clear
        The easiest way to check is to put both motors at a low power setting which is high enough for them to rotate easily, such as 30%
        Note that the fault state may be true at power up, this is normal and should clear when both motors have been driven
        For more details check the website at www.piborg.org/thunderborg and double check the wiring instructions
        """
        try:
            i2cRecv = self.RawRead(COMMAND_GET_DRIVE_B_FAULT, I2C_MAX_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading the drive fault state for motor #2!')
            return

        if i2cRecv[1] == COMMAND_VALUE_OFF:
            return False
        else:
            return True

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def GetBatteryReading(self):
        """
        voltage = GetBatteryReading()

        Reads the current battery level from the main input.
        Returns the value as a voltage based on the 3.3 V rail as a reference.
        """
        try:
            i2cRecv = self.RawRead(COMMAND_GET_BATT_VOLT, I2C_MAX_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading battery level!')
            return

        raw = (i2cRecv[1] << 8) + i2cRecv[2]
        level = float(raw) / float(COMMAND_ANALOG_MAX)
        level *= VOLTAGE_PIN_MAX
        return level + VOLTAGE_PIN_CORRECTION

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def SetBatteryMonitoringLimits(self, minimum, maximum):
        """
        SetBatteryMonitoringLimits(minimum, maximum)

        Sets the battery monitoring limits used for setting the LED colour.
        The values are between 0 and 36.3 V.
        The colours shown range from full red at minimum or below, yellow half way, and full green at maximum or higher.
        These values are stored in EEPROM and reloaded when the board is powered.
        """
        levelMin = minimum / float(VOLTAGE_PIN_MAX)
        levelMax = maximum / float(VOLTAGE_PIN_MAX)
        levelMin = max(0, min(0xFF, int(levelMin * 0xFF)))
        levelMax = max(0, min(0xFF, int(levelMax * 0xFF)))

        try:
            self.RawWrite(COMMAND_SET_BATT_LIMITS, [levelMin, levelMax])
            time.sleep(0.2) # Wait for EEPROM write to complete
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending battery monitoring limits!')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def GetBatteryMonitoringLimits(self):
        """
        minimum, maximum = GetBatteryMonitoringLimits()

        Reads the current battery monitoring limits used for setting the LED colour.
        The values are between 0 and 36.3 V.
        The colours shown range from full red at minimum or below, yellow half way, and full green at maximum or higher.
        """
        try:
            i2cRecv = self.RawRead(COMMAND_GET_BATT_LIMITS, I2C_MAX_LEN)
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed reading battery monitoring limits!')
            return

        rawMin = i2cRecv[1]
        rawMax = i2cRecv[2]
        levelMin = float(rawMin) / float(0xFF)
        levelMax = float(rawMax) / float(0xFF)
        levelMin *= VOLTAGE_PIN_MAX
        levelMax *= VOLTAGE_PIN_MAX
        return levelMin, levelMax

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def WriteExternalLedWord(self, b0, b1, b2, b3):
        """
        WriteExternalLedWord(b0, b1, b2, b3)

        Low level serial LED word writing.
        Bytes are written MSB first starting from b0
        e.g.
        WriteExtermnalLedWord(255, 64, 1, 0)
        will write out:
        11111111 01000000 00000001 00000000
        to the LEDs.
        """
        b0 = max(0, min(PWM_MAX, int(b0)))
        b1 = max(0, min(PWM_MAX, int(b1)))
        b2 = max(0, min(PWM_MAX, int(b2)))
        b3 = max(0, min(PWM_MAX, int(b3)))

        try:
            self.RawWrite(COMMAND_WRITE_EXTERNAL_LED, [b0, b1, b2, b3])
        except KeyboardInterrupt:
            raise
        except:
            self.Print('Failed sending word for the external LEDs!')

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def SetExternalLedColours(self, colours):
        """
        SetExternalLedColours([[r, g, b], ..., [r, g, b]])

        Takes a set of RGB triples to set each LED to.
        Each call will set all of the LEDs
        e.g.
        SetExternalLedColours([[1.0, 1.0, 0.0]])
        will set a single LED to full yellow.
        SetExternalLedColours([[1.0, 0.0, 0.0], [0.5, 0.0, 0.0], [0.0, 0.0, 0.0]])
        will set LED 1 to full red, LED 2 to half red, and LED 3 to off.
        """
        # Send the start marker
        self.WriteExternalLedWord(0, 0, 0, 0)

        # Send each colour in turn
        for r, g, b in colours:
            self.WriteExternalLedWord(255, 255 * b, 255 * g, 255 * r)

    # ┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈┈
    def Help(self):
        """
        Help()

        Displays the names and descriptions of the various functions and settings provided
        """
        funcList = [ThunderBorg.__dict__.get(a) for a in dir(ThunderBorg) if isinstance(ThunderBorg.__dict__.get(a), types.FunctionType)]
        funcListSorted = sorted(funcList, key = lambda x: x.func_code.co_firstlineno)

        print(self.__doc__)
        print
        for func in funcListSorted:
            print('=== {} === {}'.format(func.func_name, func.func_doc))

#EOF

