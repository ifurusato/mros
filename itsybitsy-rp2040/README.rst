****************************
I2C Slave support for RP2040
****************************

This provides some very basic I2C slave support on the RP2040, and specific
additional support for setting the color of the NeoPixel on the Adafruit
Itsy Bitsy RP2040.

The messages received may be up to 32 ASCII characters in length, with a
single byte as a return code.

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

The I2CDriver class demonstrates how to extend the I2CSlave class to perform
some kind of additional processing on the incoming payload, by overriding the
process_payload() method.

