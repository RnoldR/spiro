import os
import sys
import time
import board
import busio
import digitalio
import adafruit_vl53l1x
import adafruit_bno055

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

    
def get_vl53l1x(i2c_bus, mode, verbose = True):
    """ connects the vl53l1x to the i2c bus and returns the sensor as a result

    OPTIONAL: can set non-default values
    Mode parameter:
    1: Short mode max distance is limited to 1.3 m but better ambient immunity.
    2: Long mode can range up to 4 m in the dark with 200 ms timing budget (Default)
    """
    
    # connect vl53l1x
    vl53 = adafruit_vl53l1x.VL53L1X(i2c_bus)

    # zet parameters voor de vl53l1x
    vl53.distance_mode = mode
    vl53.timing_budget = mode * 100

    # zet de vl53 in werking
    vl53.start_ranging()

    if verbose:
        # laat wat instellingen vl53 zien
        print("--------------------")
        print('VL53L1X connected')
        if vl53.distance_mode == 1:
            print("Mode: SHORT")
        elif vl53.distance_mode == 2:
            print("Mode: LONG")
        else:
            print("Mode: UNKNOWN")
            
        print("Timing Budget: {}".format(vl53.timing_budget))
        print("--------------------")
    
    return vl53


def get_bno055(i2c_bus, verbose = True):
    bno = adafruit_bno055.BNO055_I2C(i2c_bus)
    
    # zet parameters voor de bno055
    last_val = 0xFFFF

    if verbose:
        print("------------------------------------")
        print("Sensor:       ", adafruit_bno055.__name__)
        print("Driver Ver:   ", adafruit_bno055.__version__)
        print("------------------------------------")

    return bno


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


def initialize(motors: dict) -> None:
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

    return motor_kit, led, sonar

### initialize ###

        
def joy_ride(motors, led, sensor, mqtt) -> None:
    """ Drives a robotic vehicle with simple assumptions

    Args:
        motors: instantiation of MotorKit
        led: instantiation of led
        sensor: instantiation of a distance
        mqtt: instance of mqtt server
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
                cruise_speed = int(mq_server.get_feed('set-speed'))
                
            # if

            # get distance
            distance = get_distance(sensor)
            print(f"Distance: {distance} cm - speed: {cruise_speed} - Status: {state}")
                    
            if distance > D_SAFE:
                state = 'Full_speed ahead'
                motors.move_forward(cruise_speed)
                clock_bored = time.monotonic()
                
            elif D_CAUTION < distance <= D_SAFE:
                state = 'Move carefully'
                motors.move_forward(cruise_speed / 2)
                clock_bored = time.monotonic()
                
            elif D_UNSAFE < distance <= D_CAUTION:
                # Stop and wait INTERVAL seconds
                if time.monotonic() <= (clock_bored + TIME_BORED):
                    state = 'Too close. Stop'
                    motors.move_stop()
                    
                # Look around and try to find a new, safe direction
                else:
                    state = 'Turn right'
                    motors.move_turn(50, 'right', 0.2)
                
            elif 0 <= distance <= D_UNSAFE:
                state = 'Flee! All is discovered!'
                motors.move_backward(motors.max_speed)
                time.sleep(0.2)
                clock_bored = time.monotonic()
                
            else:
                print(f'Impossible distance detected {distance}!')
                
            # if
            
            # set values in the cloud each INTERVAL seconds
            if mqtt is not None:
                if (report_clock + INTERVAL) < time.monotonic():
                    print('')
                    mqtt.set_feed('status', state)
                    mqtt.set_feed('distance', distance)

                    report_clock = time.monotonic()
                # if                
            # if
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

# motors
MOTOR_BOARD = lib_kitronik_motor.GENERIC_4WD

# other
INTERVAL = 6

# Pins
SDA = board.GP26
SCL = board.GP27

# Workshop parameters
SAVE_DISTANCE: int = 100 # value for a safe distance

# maak een I2C bus aan, let erop dat je de juiste pinnen kiest voor scl en sda
i2c = busio.I2C(scl = SCL, sda = SDA)

vl53 = get_vl53l1x(i2c, mode = 2)
bno = get_bno055(i2c)
# start driving
if __name__ == '__main__':
    print('=== Creating Connection ===')
    mq_server = create_connection()

    print('=== Starting ===')
    motors, led, sensor = initialize(MOTOR_BOARD)

    print('=== Initialized ===')
    joy_ride(motors, led, vl53, mq_server)

# if