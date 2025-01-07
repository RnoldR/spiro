import os
import sys
import time
import random

# circuitpython libraries
import busio
import board
import displayio
import microcontroller

# wifi imports
import ssl
import wifi
import ipaddress
import socketpool
import terminalio

# imports for mqtt
import adafruit_minimqtt
import adafruit_requests
from adafruit_io.adafruit_io import IO_HTTP, IO_MQTT, AdafruitIO_RequestError
from digitalio import DigitalInOut, Direction
from adafruit_httpserver.server import HTTPServer

# sensor imports
import adafruit_dht
from random import randint

# import simple cloud library
import lib_simple_cloud as cloud


# get temp and hunidity
def get_environment(sensor):
    temp = sensor.temperature
    humidity = sensor.humidity
    
    return temp, humidity
    
### get_environment ###

        
print('Start')
if __name__ == '__main__':
    INTERVAL = 6 # seconds
    
    # setup temp/humidity sensor
    dht = adafruit_dht.DHT11(board.GP28)

    #  onboard LED setup
    led = DigitalInOut(board.LED)
    led.direction = Direction.OUTPUT
    led.value = False
    
    # get clock value for timings
    clock = time.monotonic() - INTERVAL
    
    # create the mw server
    mq_server = cloud.create_connection()
    
    stopped = False
    while not stopped:
        try:
            # send data into the cloud. The INTERVAL delay prevents
            # the MQTT broker to shut down when more than 30 datapoints
            # per minute are sent
            if (clock + INTERVAL) < time.monotonic():
                # temp, humid = get_environment(dht)
                temp = random.randint(18, 40)
                humid = random.randint(40, 80)

                print(temp, humid)
                
                mq_server.set_feed('study.temperature', temp)
                mq_server.set_feed('study.humidity', humid)

                clock = time.monotonic()
                
            # if
            
        except KeyboardInterrupt:
            print('')
            print('\nStopping regstering sensor values')
            
            stopped = True
            
        except Exception as e:
            print("Error:\n", str(e))
            print("Resetting microcontroller in 10 seconds")
            
            time.sleep(10)
            microcontroller.reset()
            
        # try .. except
        
    # while
    
# if