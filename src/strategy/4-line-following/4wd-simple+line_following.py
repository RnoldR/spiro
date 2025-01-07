import os
import sys
import time
import board
import busio
import digitalio

from digitalio import DigitalInOut, Direction, Pull

import adafruit_hcsr04

#import lib_kitronik_motor
from lib_simple_cloud import SimpleMQServer, create_connection

import os
import sys
import time
import board
import busio
import digitalio
import adafruit_hcsr04
import adafruit_vl53l1x

import lib_kitronik_motor
from lib_simple_cloud import SimpleMQServer, create_connection


def get_distance(dist_sensor: Object) -> int:
    """ Determines distance with HC-SR04

    Args:
        dist_sensor (Object): Reference to Adafruit instance of HC-SR04

    Raises:
        RuntimeError: Raised when more than 10 successive errors occur

    Returns:
        int: distance in centimeters
    """
    error_count = 0
    
    while True:
        try:
            distance = dist_sensor.distance
            
            # found some distance, return it as an integer value
            return int(distance)
        
        # runtime error: probably distance too far, try again
        except RuntimeError:
            error_count += 1
            if error_count > 10:
                raise RuntimeError('Sonar detected more than 10 successive errors')
            
        # try..except
        
    # while

### get_distance ###

    
def create_connection():
    # create the simple server
    mq_server = SimpleMQServer(os.getenv('WIFI_SSID'), os.getenv('WIFI_PASSWORD'))

    # get an mqtt object
    aio_username = os.getenv('ADAFRUIT_IO_USERNAME')
    aio_key = os.getenv('ADAFRUIT_IO_KEY')
    io = mq_server.create_mqtt_connection(aio_username, aio_key)
    
    # connect to the feeds
    mq_server.create_feed('status')
    mq_server.create_feed('distance')

    print("Data feeds created")
    
    return mq_server

### create_connection ###


def initialize(motors: dict, trigger_pin, echo_pin) -> None:
    """ Initializes variables

    Args:
        motors (dict): contains a motor definition dictionary
        trigger_pin: pin to HC-SR04+ trigger pin
        echo_pin: pin to HC-SR04+ echo pin

    Returns:
        - instantiation of motor kit
        - instantiation of a led that can be used 
        - HC-SR04+ sensor
    """
    # create motor driver
    motor_kit = lib_kitronik_motor.MotorKit(motors)

    motor_kit.set_motors(False)
    motor_kit.set_max_speed(MAX_SPEED)
    motor_kit.set_verbose(False)
    motor_kit.move_turn(0, 'straight')

    # onboard LED setup 
    led = digitalio.DigitalInOut(board.LED)
    led.direction = digitalio.Direction.OUTPUT
    led.value = False

    # get sonar distance sensor
    sonar = adafruit_hcsr04.HCSR04(trigger_pin=trigger_pin, echo_pin=echo_pin)
    
    # IR sensors
    ir_right = DigitalInOut(IR_RIGHT_PIN)
    ir_right.direction = Direction.INPUT

    ir_left = DigitalInOut(IR_LEFT_PIN)
    ir_left.direction = Direction.INPUT

    return motor_kit, led, sonar, ir_right, ir_left

### initialize ###


def joy_ride(motors, led, sensor, mqtt, ir_right, ir_left) -> None:
    """ Drives a robotic vehicle with simple assumptions

    Args:
        motors: instantiation of MotorKit
        led: instantiation of led
        sensor: instantiation of a distance
        mqtt: instance of mqtt server
        right: ir_right IT sensor
        left: IR_left IR sensor
    """
    # everything setup: motors can be started
    motors.set_motors(True)
    motors.set_verbose(True)
    state: str = 'Driving'
    clock_bored = time.monotonic()
    report_clock = time.monotonic()

    try:
        cruise_speed = motors.max_speed
        
        while True:
            if mqtt is not None:
                # get the values of on-off feed from adafruit cloud
                onoff = bool(int(mqtt.get_feed('on-off')))
                motors.set_motors(onoff)
                led.value = onoff

                # get_speed
                cruise_speed = int(mq_server.get_feed('speed'))
                
            # if
            
            # acquire the states of the two IR sensors
            # False = bright surface, False = dark surface
            right = ir_right.value
            left = ir_left.value
            
            print('IR sensors:', right, left)
            # when both sensors are bright => ok
            if not right and not left:
                continue
            
            # both are black, unsolvable situation
            elif right and left:
                print('Unsolvable situation, quitting...')
                motors.move_stop()
                break
            
            #  right sensor detects dark, turn right
            elif right and not left:
                motors.move_turn(50, 'right', 0.2)

            #  left sensor detects dark, turn left
            elif not right and left:
                motors.move_turn(50, 'left', 0.2)

        # while
        
    except KeyboardInterrupt:
        pass

    finally:
        state = 'Final Stop'
        led.value = False
        motors.move_stop()

        print(f'=== {state} ===')

    # try..finally
    
    return

### joy_ride ###


# set motor constants
MAX_SPEED: int = 30
D_SAFE: int = 80
D_CAUTION: int = 50
D_UNSAFE: int = 30
TIME_BORED: int = 3

# Pins
TRIGGER_PIN = board.GP14
ECHO_PIN = board.GP15
IR_RIGHT_PIN = board.GP17
IR_LEFT_PIN = board.GP16

# motors
MOTOR_BOARD = lib_kitronik_motor.GENERIC_4WD

# other
INTERVAL = 6

# start driving
if __name__ == '__main__':
    print('=== Creating Connection ===')
    mq_server = create_connection()

    print('=== Starting ===')
    motors, led, sensor, right, left = initialize(MOTOR_BOARD, trigger_pin = TRIGGER_PIN, echo_pin = ECHO_PIN)

    print('=== Initialized ===')
    joy_ride(motors, led, sensor, mq_server, right, left)

# if