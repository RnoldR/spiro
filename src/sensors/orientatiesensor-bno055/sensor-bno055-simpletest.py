# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import sys
import time
import board
import busio
import adafruit_bno055

# get the i2c bus
i2c = busio.I2C(scl = board.GP17, sda = board.GP16)

# fetch sensor bno055
sensor = adafruit_bno055.BNO055_I2C(i2c)
sensor.mode = adafruit_bno055.NDOF_MODE

print(dir(sensor))

heading = 180

while True:
    print(f"Accelerometer: {sensor.acceleration} (m/s^2)")
    print(f"Magnetometer: {sensor.magnetic} (microteslas)")
    print(f"Gyroscope: {sensor.gyro} (rad/sec)")
    print(f"Euler angle: {sensor.euler}")
    print(f"Quaternion: {sensor.quaternion}")
    print(f"Linear acceleration: {sensor.linear_acceleration} (m/s^2)")
    print(f"Gravity: {sensor.gravity} (m/s^2)")
    direction = sensor.euler[0]
    deviation = min (direction - 360, direction)
    print(f'Direction: {direction} Heading: {heading} Deviation: {deviation}')
    print('')

    time.sleep(2)
