# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2021 Carter Nelson for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense

# Simple demo of the VL53L1X distance sensor.
# Will print the sensed range/distance every second.

import time
import board
import busio
import adafruit_bno055
import adafruit_vl53l1x

# maak een I2C bus aan, let erop dat je de juiste pinnen kiest voor scl en sda
i2c = busio.I2C(scl = board.GP15, sda = board.GP14)

# en verbindt de vl53l1x
vl53 = adafruit_vl53l1x.VL53L1X(i2c)

print("--------------------")
print('VL53L1X connected')
# OPTIONAL: can set non-default values
# Mode parameter:
# 1: Short mode max distance is limited to 1.3 m but better ambient immunity.
# 2: Long mode can range up to 4 m in the dark with 200 ms timing budget (Default)
vl53.distance_mode = 2
vl53.timing_budget = 200
if vl53.distance_mode == 1:
    print("Mode: SHORT")
elif vl53.distance_mode == 2:
    print("Mode: LONG")
else:
    print("Mode: UNKNOWN")

print("Timing Budget: {}".format(vl53.timing_budget))
print("--------------------")

# connect bno055
bno = adafruit_bno055.BNO055_I2C(i2c)

last_val = 0xFFFF

vl53.start_ranging()

while True:
    distance = -1
    if vl53.data_ready:
        distance = vl53.distance
        vl53.clear_interrupt()
        time.sleep(1.0)
        
    euler = bno.euler
    direction = euler[0]
    print(f'Distance:  {distance} cm')
    print(f'Direction: {direction} degrees')
    
    time.sleep(2)
