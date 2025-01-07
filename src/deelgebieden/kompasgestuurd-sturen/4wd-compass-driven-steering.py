import sys
import time
import board
import busio
import digitalio
import adafruit_bno055
import adafruit_vl53l1x
import lib_kitronik_motor


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
                raise RuntimeError('Distance sensor got more than 10 successive errors')
            
        # try..except
        
    # while

### get_distance ###


def initialize(motors: dict):
    """ Initializes variables

    Args:
        motors (dict): contains a motor definition dictionary

    Returns:
        - instantiation of motor kit
        - instantiation of a led that can be used 
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

    return motor_kit, led

### initialize ###

        
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


def joy_ride(motors, led, dist_sensor, bno_sensor) -> None:
    """ Drives a robotic vehicle with simple assumptions

    Args:
        motors: instantiation of MotorKit
        led: instantiation of led
        dist_sensor: instantiation of a distance
    """
    # everything setup: motors can be started
    motors.set_motors(True)
    state: str = 'Driving'
    clock_bored = time.monotonic()

    try:
        cruise_speed = motors.max_speed
        while True:
            distance = get_distance(dist_sensor)
            
            print(f"Status: {state} Distance: {distance} cm ")
                    
            if distance >= SAVE_DISTANCE:
                state = 'Full_speed ahead'
                led.value = True
                motors.move_forward(cruise_speed)
                
            else:
                state = 'Turn Right'
                led.value = True
                motors.move_turn(cruise_speed, 'right', 0.1)

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

# motors
MOTOR_BOARD = lib_kitronik_motor.GENERIC_4WD

# Pins
SDA = board.GP26
SCL = board.GP27

# Workshop parameters
SAVE_DISTANCE: int = 100 # value for a safe distance

# maak een I2C bus aan, let erop dat je de juiste pinnen kiest voor scl en sda
i2c = busio.I2C(scl = SCL, sda = SDA)

vl53 = get_vl53l1x(i2c, mode = 2)
bno055 = get_bno055(i2c)

# start driving
if __name__ == '__main__':
    print('=== Starting ===')
    motors, led = initialize(MOTOR_BOARD)

    print('=== Initialized ===')
    joy_ride(motors, led, vl53, bno055)
