# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
"""
From the Adafruit Q&A:
(https://cdn-learn.adafruit.com/downloads/pdf/adafruit-bno055-absolute-orientation-sensor.pdf)

Bosch is the first company to get this right by taking a MEMS accelerometer,
magnetometer and gyroscope and putting them on a single die with a high speed
ARM Cortex-M0 based processor to digest all the sensor data, abstract the sensor
fusion and real time requirements away, and spit out data you can use in
quaternions, Euler angles or vectors.

I2C problems
The BNO055 I2C implementation violates the I2C protocol in some
circumstances. This causes it not to work well with certain chip families.
It does not work well with Espressif ESP32, ESP32-S3, and NXP i.MX RT1011,
and it does not work well with I2C multiplexers. Operation with SAMD51,
RP2040, STM32F4, and nRF52840 is more reliable.

Calibration
Another thing to be aware of is that until the sensor calibrates
it has a relative orientation output (i.e. orientation will
be relative to where the sensor was when it powered on).

A system status value of '0' in NDOF mode means that the device
has not yet found the 'north pole', and orientation values will
be relative not absolute. Once calibration and setup is complete
(system status > '0') the heading will jump to an absolute
value since the BNO has found magnetic north (the system calibration
status jumps to 1 or higher). See the Device Calibration page
in this learning guide for further details.

Why doesn't Euler output seem to match the Quaternion output?
The Euler angles coming out of the chip are based on 'automatic
orientation detection', which has the drawback of not being
continuous for all angles and situations.

According to Bosch BNO055 Euler angle output should only
be used for eCompass, where pitch and roll stay below 45 degrees.
For absolute orientation, quaternions should always be used,
and they can be converted to Euler angles at the last moment
via the euler_from_quaternion helper function.
"""
import sys
import math
import time
import busio
import board
import adafruit_bno055


def temperature():
    """
    improved way of computing the temperature, well, alleged, that is.
    """
    global last_val  # pylint: disable=global-statement
    result = sensor.temperature
    if abs(result - last_val) == 128:
        result = sensor.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
    last_val = result
    
    return result

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians


"""
Due to electro-magnetic fields being generated from
ferro-metals and electricity the compass will never
be well calibrated. Calibration helps, below
some calibration values.

Run the bno055-calibrator on a regular basis.
"""

# I2C Pins
PIN_SDA = board.GP26
PIN_SCL = board.GP27

i2c = busio.I2C(scl = PIN_SCL, sda = PIN_SDA)
bno055 = adafruit_bno055.BNO055_I2C(i2c)

last_val = 0xFFFF

bno055.offsets_magnetometer =  (258, -418, -331)
bno055.offsets_gyroscope =     (0, -2, 0)
bno055.offsets_accelerometer = (-34, -66, -33)

print("------------------------------------")
print("Sensor:       ", adafruit_bno055.__name__)
print("Driver Ver:   ", adafruit_bno055.__version__)
print("------------------------------------")

while True:
    # Uncomment code below when using a Raspberry Pi
    #print("Temperature: {} degrees C".format(temperature()))

    print(f"Temperature: {bno055.temperature} degrees C")
    q = bno055.quaternion
    new_euler = euler_from_quaternion(q[0], q[1], q[2], q[3])
    new_euler = [x * 57.32 for x in new_euler]
    print(f"Accelerometer (m/s^2): {bno055.acceleration}")
    #print(f"Magnetometer (microteslas): {bno055.magnetic)}")
    print(f"Gyroscope (rad/sec): {bno055.gyro}")
    print(f"Euler angle: {bno055.euler}")
    print(f"New Euler:   {new_euler}")
    print(f"Quaternion:  {bno055.quaternion}")
    print(f"Linear acceleration (m/s^2): {bno055.linear_acceleration}")
    print(f"Gravity (m/s^2): {bno055.gravity}")
    print(f"Heading in degrees: {bno055.euler[0]} ({new_euler[0]})")
    print()

    time.sleep(3)
