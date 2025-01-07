import gc
import os
import sys
import ssl
import time
import wifi
import board
import busio
import digitalio
import ipaddress
import socketpool
import adafruit_hcsr04
#import adafruit_vl53l1x

from digitalio import DigitalInOut, Direction
from adafruit_httpserver.request import HTTPRequest
from adafruit_httpserver.response import HTTPResponse
from adafruit_httpserver.methods import HTTPMethod
from adafruit_httpserver.mime_type import MIMEType

import adafruit_requests
from adafruit_io.adafruit_io import IO_HTTP, AdafruitIO_RequestError

import lib_simple_server as lss
import lib_kitronik_motor

MAX_SPEED: int = 50
D_SAFE: int = 100
D_CAUTION: int = 50
D_UNSAFE: int = 30

bot_4wd = {'motors': {'left_back': 4,
                      'left_front': 3,
                      'right_front': 1,
                      'right_back': 2,
                     },
           'servo': None,
          }

bot_4wd_steer = {'motors': {'left_back': 4,
                            'right_back': 3,
                           },
                 'servo': {'servo': 1,
                           'right': 45,
                           'neutral': 104,
                           'left': 135,
                          }
                 }



# create motor drive
motor_driver = lib_kitronik_motor.MotorKit(bot_4wd_steer['motors'],
                                           bot_4wd_steer['servo'],
                                          ) 

motor_driver.set_max_speed(MAX_SPEED)
motor_driver.set_motors(False)
motor_driver.set_verbose(False)

state: str = 'Motors turned off'

def handle_on(motors):
    motors.set_motors(True)
    led.value = True
    print('Motors enabled')
    
    return
    
### handle_on ###
    

def handle_off(motors):
    motors.set_motors(False)
    led.value = False
    print('Motors disabled')
    
    return
    
### handle_off ###


def handle_forward(motors):
    motors.move_forward(motors.max_speed)
    
    return
    
### handle_off ###


def handle_backward(motors):
    motors.move_backward(motors.max_speed)
    
    return
    
### handle_off ###


def handle_turnleft(motors):
    motors.move_turn(motors.max_speed / 2, 'left', 0.5)
    
    return
    
### handle_off ###


def handle_turnright(motors):
    motors.move_turn(motors.max_speed / 2, 'right', 0.5)
    
    return
    
### handle_off ###


def handle_stop(motors):
    motors.move_stop()
    
    return
    
### handle_off ###


def get_distance_vl53(dist_sensor):
    # start the distance sensor
    dist_sensor.start_ranging()

    # when there is data, read it
    while not vl53.data_ready:
        pass

    distance = dist_sensor.distance
    
    # when None is returned, usually the object is too far
    # just assume the upper working limit
    if distance is None:
        distance = 400
        
    dist_sensor.clear_interrupt()
        
    return distance

### get_distance_vl53 ###


def get_distance_sonar(dist_sensor):
    try:
        distance = dist_sensor.distance
        
    except RuntimeError:
        distance = 401
        
    # try..except
    
    if distance > 401:
        distance = 401
        
    # if
    
    return distance

### get_distance_sonar ###


def get_distance():
    if vl53 is not None:
        # get the sensors from the multiplexer
        vl53_front = adafruit_vl53l1x.VL53L1X(mux[7]) # front sensor
        vl53_rear = adafruit_vl53l1x.VL53L1X(mux[4]) # rear sensor
        
        front = get_distance(vl53_front)
        read = get_distance(vl53_rear)
        
        return front, rear # get_distance_vl53(vl53)
    
    if sonar is not None:
        return get_distance_sonar(sonar)
    
    raise ValueError('All distance sensors are None')

### get_distance ###


def get_unbored(speed: int):
    distance = get_distance()
    
    while distance < 50:
        motor_driver.move_turn(speed / 2, 'right', 0.1)
        distance = get_distance()
        
    # while

    motor_driver.move_turn(speed / 2, 'right', 0.25)
    
### get_unbored ###
        
    
# onboard LED setup 
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT
led.value = False

# define IR line following sensors
left_ir = digitalio.DigitalInOut(board.GP14)
left_ir.direction = digitalio.Direction.INPUT
right_ir = digitalio.DigitalInOut(board.GP15)
right_ir.direction = digitalio.Direction.INPUT

# define IR wheel sensor
wheel_left_back = digitalio.DigitalInOut(board.GP3)
wheel_left_back.direction = digitalio.Direction.INPUT

vl53 = None
try:
    # get the i2c bus
    i2c = busio.I2C(board.GP19, board.GP18)

    led.value = True

    # load mosensors when modules are imported
    if 'adafruit_vl53l1x' in sys.modules:
        # use it to get the distance sensor
        vl53 = adafruit_vl53l1x.VL53L1X(i2c)
        vl53.distance_mode = 2
        vl53.timing_budget = 100
        
    # if
    
except RuntimeError:
    i2c = None
    print('*** No I2C bus present, ignored ***')
    
### try..except
    
if 'adafruit_hcsr04' in sys.modules:
    # get sonar distance
    sonar = adafruit_hcsr04.HCSR04(trigger_pin=board.GP13, echo_pin=board.GP12)

else:
    sonar = None
    
# if

# create the simple server
server = lss.SimpleServer(os.getenv('WIFI_SSID'), os.getenv('WIFI_PASSWORD'))

# set up MQTT connection
requests = adafruit_requests.Session(server.pool, ssl.create_default_context())
print('Requests:', requests)

# Initialize an Adafruit IO HTTP API object
aio_username = os.getenv('ADAFRUIT_IO_USERNAME')
aio_key = os.getenv('ADAFRUIT_IO_KEY')
io = IO_HTTP(aio_username, aio_key, requests)

# Set up the callback methods 
#io.on_connect = connected
#io.on_message = message

# connect to the feeds
distance_feed = io.get_feed("distance")
status_feed = io.get_feed("status")

#  pack feed names into an array for the loop
feed_names = [status_feed, distance_feed]
print("Data feeds created")

# address to ping to determine if connection is alive
ping_address = ipaddress.ip_address("8.8.4.4")

# get clock value for timings
clock = time.monotonic() #  time.monotonic() holder for server ping

# keep track when the car is getting bored
clock_bored = time.monotonic()

# set clock for sending data
clock_data = time.monotonic()

handle_off(motor_driver)

try:
    cruise_speed = motor_driver.max_speed
    while True:
        """
        #  every 30 seconds, ping server & update temp reading
        if (clock + 30) < time.monotonic():
            
            if wifi.radio.ping(ping_address) is None:
                print("lost connection")
                
            else:
                print("connected")
                
            # if
                
            clock = time.monotonic()

        # if
        
        #  poll the server for incoming/outgoing requests
        server.server.poll()
        #continue
        """
        
        distance = get_distance()
        direction = 180
        
        print(f"Distance: {distance} cm Direction: {direction} degrees RAM: {gc.mem_free()} KB")
                
        # get the values of a feed as a dictionary, just get the value
        onoff = int(io.receive_data('on-off')['value'])
        speed = int(io.receive_data('set-speed')['value'])
        #if speed < 10:
        #    speed = 0
            
        print('On/off:', onoff, 'speed:', speed)
        
        # set the internal state to the the input
        motor_driver.set_motors(bool(onoff))
        cruise_speed = speed
        motor_driver.set_max_speed(cruise_speed)

        if (clock + 2) < time.monotonic():
            #  read sensor
            data = [state, distance]
            
            #  send sensor data to respective feeds
            for z in range(len(data)):
                io.send_data(feed_names[z]["key"], data[z])
                print(f"sent {data[z]}")
                time.sleep(1)
                
            # for
                
            print()
            time.sleep(1)
            
            #  reset clock
            clock = time.monotonic()
            
        # if            
    
        if distance is None:
            clock_bored = time.monotonic()
            continue
        
        elif distance > D_SAFE:
            state = 'Full_speed ahead'
            motor_driver.move_forward(cruise_speed)
            clock_bored = time.monotonic()
            
        elif distance > D_CAUTION and distance <= D_SAFE:
            state = 'Move carefully'
            motor_driver.move_forward(cruise_speed / 2)
            clock_bored = time.monotonic()
            
        elif distance > D_UNSAFE and distance <= D_CAUTION:
            state = 'Stop!'
            motor_driver.move_stop()
            if (clock_bored + 3) < time.monotonic():
                get_unbored(cruise_speed)
                clock_bored = time.monotonic()
            
        else:
            state = 'Flee!'
            motor_driver.move_backward(motor_driver.max_speed)
            time.sleep(1)
            clock_bored = time.monotonic()
            
        # if
        
    # while

finally:
    state = 'Final Stop'
    motor_driver.move_stop()
    led.value = False

# try..finally