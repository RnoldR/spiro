import sys
import time
import board
import busio
import digitalio
import adafruit_hcsr04
import adafruit_vl53l1x
import lib_kitronik_motor


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


def initialize(motors: dict, scl, sda) -> None:
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
    #sonar = adafruit_hcsr04.HCSR04(trigger_pin=board.GP19, echo_pin=board.GP18)

    # get vl53l1x sensor
    vl53 = None
    try:
        # get the i2c bus
        i2c = busio.I2C(scl = scl, sda = sda)

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
        led.value = None
        print('*** No I2C bus present, ignored ***')
        
    ### try..except
        
        
    return motor_kit, led, vl53

### initialize ###

        
def joy_ride(motors, led, sensor) -> None:
    """ Drives a robotic vehicle with simple assumptions

    Args:
        motors: instantiation of MotorKit
        led: instantiation of led
        sensor: instantiation of a distance
    """
    # everything setup: motors can be started
    motors.set_motors(True)
    state: str = 'Driving'
    clock_bored = time.monotonic()

    try:
        cruise_speed = motors.max_speed
        while True:
            distance = get_distance_vl53(sensor)
            
            print(f"Distance to nearest object in front: {distance} cm ")
                    
            # comment out "continue" when distances are correct
            #continue
        
            if distance > D_SAFE:
                state = 'Full_speed ahead'
                led.value = True
                motors.move_forward(cruise_speed)
                clock_bored = time.monotonic()
                
            elif D_CAUTION < distance <= D_SAFE:
                state = 'Move carefully'
                led.value = True
                motors.move_forward(cruise_speed / 2)
                clock_bored = time.monotonic()
                
            elif D_UNSAFE < distance <= D_CAUTION:
                state = 'Stop!'
                led.value = False
                motors.move_stop()
                if (clock_bored + TIME_BORED) < time.monotonic():
                    motors.move_turn(-cruise_speed, 'left', 0.2)
                    motors.move_turn(cruise_speed, 'straight', 0)
                
            elif 0 <= distance <= D_UNSAFE:
                state = 'Flee! All is discovered!'
                led.value = False
                motors.move_backward(motors.max_speed)
                time.sleep(1)
                clock_bored = time.monotonic()
                
            else:
                print(f'Impossible distance detected {distance}!')
                
            # if
            
        # while
        
    except KeyboardInterrupt:
        pass

    finally:
        state = 'Final Stop'
        led.value = False
        motors.move_stop()
        led.value = False
        print('=== Stopped ===')

    # try..finally
    
    return

### joy_ride ###


# set motor constants
MAX_SPEED: int = 50
D_SAFE: int = 100
D_CAUTION: int = 50
D_UNSAFE: int = 30
TIME_BORED: int = 3

# I2C pins
PIN_SCL: int = board.GP15
PIN_SDA: int = board.GP14

# motors
MOTOR_BOARD = lib_kitronik_motor.GENERIC_4WD

# start driving
if __name__ == '__main__':
    print('=== Starting ===')
    motors, led, sensor = initialize(MOTOR_BOARD, scl = PIN_SCL, sda = PIN_SDA)

    print('=== Initialized ===')
    joy_ride(motors, led, sensor)
