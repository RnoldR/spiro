# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2021 Carter Nelson for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense

# Simple demo of the VL53L1X distance sensor.
# Will print the sensed range/distance every second.

import time
import board
import busio
import adafruit_vl53l1x

PIN_SCL = board.GP27
PIN_SDA = board.GP26

# maak een I2C bus aan
i2c = busio.I2C(scl = PIN_SCL, sda = PIN_SDA)

# en verbindt de vl53l1x
vl53 = adafruit_vl53l1x.VL53L1X(i2c)

# OPTIONAL: can set non-default values
vl53.distance_mode = 2
vl53.timing_budget = 200

print("VL53L1X Simple Test.")
print("--------------------")

# OPTIONAL: can set non-default values
# Mode parameter:
# 1: Short mode max distance is limited to 1.3 m but better ambient immunity.
# 2: Long mode can range up to 4 m in the dark with 200 ms timing budget (Default)
#vl53.distance_mode = 2
#vl53.timing_budget = 200

model_id, module_type, mask_rev = vl53.model_info
print("Model ID: 0x{:0X}".format(model_id))
print("Module Type: 0x{:0X}".format(module_type))
print("Mask Revision: 0x{:0X}".format(mask_rev))
print("Distance Mode: ", end="")

if vl53.distance_mode == 1:
    print("SHORT")
elif vl53.distance_mode == 2:
    print("LONG")
else:
    print("UNKNOWN")

print("Timing Budget: {}".format(vl53.timing_budget))
print("--------------------")

vl53.start_ranging()

while True:
    if vl53.data_ready:
        print("Distance: {} cm".format(vl53.distance))
        vl53.clear_interrupt()
        time.sleep(1.0)
