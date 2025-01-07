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


def get_distance_sonar(dist_sensor: Object) -> int:
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
            
            # found some distance, return it
            return int(distance)
        
        # runtime error: probably distance too far, try again
        except RuntimeError:
            error_count += 1
            if error_count > 10:
                raise RuntimeError('Sonar detected more than 10 errors')
            
        # try..except
        
    # while

### get_distance_sonar ###
    
    
def get_distance_vl53(sensor: Object) -> int:
    """ Determines distance with vl53l1x

    Args:
        dist_sensor (Object): Reference to Adafruit instance of vl53l1x. When
            None is returned it is converted to 401, 1 more than the maximumn
            reliable distance.

    Returns:
        int: distance in centimeters
    """
    sensor.start_ranging()

    sensor.start_ranging()
    while not sensor.data_ready:
        pass
            
    #sensor.clear_interrupt()
    
    distance = sensor.distance
    if distance is None:
        distance = 401
        
    return int(distance)

### get_distance_vl53 ###


def get_distance(sensor: Object) -> int:
    """ Returns the distance in cm depending on type of sensor.

    Args:
        sensor (Object): sensor to measure distance.

    Returns:
        int: distance in cm.
    """
    if isinstance(sensor, adafruit_vl53l1x.VL53L1X):
        distance = get_distance_vl53(sensor)
        
    elif isinstance(sensor, adafruit_hcsr04.HCSR04):
        distance = get_distance_sonar(sensor)

    else:
        print('Unknown sensor')
        
    return distance

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


def initialize(motors: dict, sensor = None, scl = None, sda = None, trigger = None, echo = None) -> None:
    """ Initializes variables

    Args:
        motors (dict): contains a motor definition dictionary
        scl: pin to i2c clock
        sda: pin to sda data

    Returns:
        - instantiation of motor kit
        - instantiation of a led that can be used 
        - vl53l1x sensor
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

    if sensor == "HCSR04":
        device = adafruit_hcsr04.HCSR04(trigger_pin=trigger, echo_pin=echo)

    elif sensor == "VL53L1X":
        # get the i2c bus
        i2c = busio.I2C(scl = scl, sda = sda)

        # get vl53l1x sensor
        device = adafruit_vl53l1x.VL53L1X(i2c)
        device.distance_mode = 2
        device.timing_budget = 100
                
    else:
        raise RuntimeError(f'Unknown sensor in initialize: {sensor}')
        
    # if
        
    return motor_kit, led, device

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
D_SAFE: int = 50
D_CAUTION: int = 50
D_UNSAFE: int = 30
TIME_BORED: int = 3

# I2C pins
PIN_SCL: int = board.GP15
PIN_SDA: int = board.GP14
PIN_TRIGGER: int = board.GP15
PIN_ECHO: int = board.GP14

# motors
MOTOR_BOARD = lib_kitronik_motor.GENERIC_4WD

# other
INTERVAL = 6

# start driving
if __name__ == '__main__':
    print('=== Starting ===')
    mq_server = None # create_connection()

    motors, led, sensor = initialize(MOTOR_BOARD,
                                     sensor = "HCSR04",
                                     trigger = PIN_TRIGGER,
                                     echo = PIN_ECHO)

    print('=== Initialized ===')
    joy_ride(motors, led, sensor, mq_server)

# if