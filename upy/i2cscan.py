# Scanner i2c en MicroPython | MicroPython i2c scanner
# Renvoi l'adresse en decimal et hexa de chaque device connecte sur le bus i2c
# Return decimal and hex address of each i2c device
# https://projetsdiy.fr - https://diyprojects.io (dec. 2017)

import machine

# Zio Pyboard Qwiic1 PB6-SCL1-X9
# Zio Pyboard Qwiic1 PB7-SDA1-X10

# Zio Pyboard Qwiic2 PB10-SCL2-Y9
# Zio Pyboard Qwiic2 PB11-SDA2-Y10

print('creating i2c connection…')

#_pin_scl = machine.Pin('B6')
#_pin_sda = machine.Pin('B7')

print('creating i2c connection…')
i2c = machine.I2C(1)

print('scanning i2c bus…')
devices = i2c.scan()

if len(devices) == 0:
    print("no i2c devices found!")
else:
    print('{} i2c devices found.'.format(len(devices)))

for device in devices:  
    print("decimal address: {}; hex address: {}".format(device, hex(device)))

#EOF
