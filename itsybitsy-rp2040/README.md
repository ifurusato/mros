****************************
I2C Slave support for RP2040
****************************

This provides some very basic I2C slave support on the RP2040, and specific
additional support for setting the color of the NeoPixel on the Adafruit
Itsy Bitsy RP2040. The code is written in MicroPython.

Messages may be up to 32 ASCII characters in length, with a single byte as
a return code.

The send.py file is a demo I2C master for the I2CDriver, e.g.:

  ```
  % send.py "set dark_orange"
  ```

This sets the RGB LED on the Itsy Bitsy RP2040 to dark orange.

The specific files used include:

* i2c_slave.py  : the core I2CSlave class
* i2c_driver.py : extends I2CSlave class to set the NeoPixel
* RP2040_Slave.py : the actual I2C implementation
* RP2040_I2C_Registers.py : used by RP2040_Slave
* send.py : a demo I2C Master, used to send messages

* i2c_slave_test.py : tests just the I2CSlave class
* i2c_driver_test.py : tests the I2CDriver class (copied as main.py)

Additional utility classes:

* colors.py : an enumeration of some RGB color values
* itertools.py : a cheap copy of itertools
* neopixel.py  : NeoPixel support
* stringbuilder.py : similar to Java's class

Untested, work-in-progress files:

* sensor.py
* sensor_test.py

The `I2CDriver` class demonstrates how to extend the `I2CSlave` class to perform
some kind of additional processing on the incoming payload, by overriding the
`process_payload()` method.


Requirements
************

This was programmed using MicroPython v1.23.0 but will probably work in older
versions as it doesn't use any of the newer language features.


Installation
************

If you're installing the I2CSlave on a generic RP2040 and don't require the
additional I2CDriver class, the only required files are:

* i2c_slave.py
* RP2040_I2C_Registers.py
* RP2040_Slave.py
* stringbuilder.py
* colors.py
* itertools.py

and your main.py file, which could be based on a copy of i2c_slave_test.py. If
you have no need for displaying colors, references to the color.py (used by the
status() method) could be commented out or removed as they are not necessary
for basic functionality.


Support & Liability
*******************

This project comes with no promise of support or acceptance of liability. Use at
your own risk.


Copyright & License
*******************

All contents (including software, documentation and images) Copyright 2020-2024
by Murray Altheim. All rights reserved.

Software and documentation are distributed under the MIT License, see LICENSE
file included with MR01 project.

