import time
import board
import busio

def get_i2c_devices(i2c):
    # returns a list of i2c addresses connected to i2c
    while not i2c.try_lock():
        pass

    # initialize an empty list
    devices = []
    try:
        # add devices to list
        devices = [device_address for device_address in i2c.scan()]

    finally:  # unlock the i2c bus when ctrl-c'ing out of the loop
        i2c.unlock()
    
    # try..finally
    
    return devices

### get_i2c_devices ###

def get_i2c_str(i2c):
    # return a list of i2c devices in hexadecimal string format
    devices = get_i2c_devices(i2c)
    
    devices_as_string = [hex(device) for device in devices]
    
    return devices_as_string

### get_i2c_str ###


if __name__ == '__main__':
    #################### Create I2C bus ####################
    PIN_SDA = board.GP14
    PIN_SCL = board.GP15

    i2c_bus = busio.I2C(scl=PIN_SCL, sda=PIN_SDA)

    while True:
        i2c_devs = get_i2c_devices(i2c_bus)
        
        print('I2C devices found (int):', i2c_devs)
        
        print('I2C devices found (hex):', get_i2c_str(i2c_bus))
        
        time.sleep(2)
        
    # while

