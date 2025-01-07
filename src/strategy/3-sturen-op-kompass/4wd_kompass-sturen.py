import sys
import time
import board
import busio
import digitalio
import adafruit_bno055
import adafruit_vl53l1x
import lib_kitronik_motor


def get_vl53_sensor(i2c_bus: object):
    """ Gets a VL53L1X sensor driver and attaches it to the I2C bus

    Vl53l1x has some optional parameters.
    distance_mode:
    1: Short mode max distance is limited to 1.3 m but better ambient immunity.
    2: Long mode can range up to 4 m in the dark with 200 ms timing budget (Default)

    timing_budget:
    number of ms between measurements. 20 ms is the minimum required for 
    short distance (mode = 1) and 140 ms is minimally required when using
    the full 4m distance ranging. See the data sheet
    https://cdn-learn.adafruit.com/assets/assets/000/105/859/original/vl53l1x.pdf?1634930163

    Args:
        i2c_bus (object): I2C bus to attach the sensor to

    Returns:
        object: the object representing the Vl53L1X sensor
    """
    # get and attach the sensor
    vl53 = adafruit_vl53l1x.VL53L1X(i2c_bus)

    # set long range mode and minimal required timing budget
    vl53.distance_mode = 2
    vl53.timing_budget = 200
    text_mode = "UNKNOWN"
    if vl53.distance_mode == 1:
        print("SHORT")
    elif vl53.distance_mode == 2:
        print("LONG")

    # get some info about the sensor
    model_id, module_type, mask_rev = vl53.model_info

    # print the info
    print("VL53L1X Simple Test.")
    print("--------------------")
    print(f"Model ID: 0x{model_id:0X}")
    print(f"Module Type: 0x{module_type:0X}")
    print(f"Mask Revision: 0x{mask_rev:0X}")
    print(f"Timing Budget: {vl53.timing_budget} ms ({text_mode})")
    print("--------------------")
    
    return vl53

### get_vl53_sensor ###


def get_bno055_sensor(i2c_bus: object):
    bno055 = adafruit_bno055.BNO055_I2C(i2c)

    bno055.offsets_magnetometer =  (258, -418, -331)
    bno055.offsets_gyroscope =     (0, -2, 0)
    bno055.offsets_accelerometer = (-34, -66, -33)

    print("------------------------------------")
    print("Sensor:       ", adafruit_bno055.__name__)
    print("Driver Ver:   ", adafruit_bno055.__version__)
    print("------------------------------------")
    
    return bno055

### get_bno055_sensor ###


def get_distance(dist_sensor: object) -> int:
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
            if distance is None:
                return 100
            else:
                return int(distance)
                
        # runtime error: probably distance too far, try again
        except RuntimeError:
            error_count += 1
            if error_count > 10:
                raise RuntimeError('Sonar detected more than 10 successive errors')
            
        # try..except
        
    # while

### get_distance ###
    
    
def get_heading(orientation_sensor: object):
    heading = orientation_sensor.euler[0]
    
    return set_heading(heading)
    
### get_heading ###


def set_heading(heading: float):
    while heading < 0:
        heading += 360
        
    while heading > 360:
        heading -= 360
    
    return heading

### set_heading ###


def correct_heading(
        orientation_sensor: object, 
        desired_heading: float, 
        allowed_error: float,
    ) -> float:
    """ Position the robot into the position as requested 
        by the desired_heading parameter

    Because of mechanical imperfections the desired_heading 
    will never be exactly reached and the robot will endlessly
    try to find the correct heading. Therefore an error margin
    has been built-in: the correct heading lies between
    desired_heading +- allowed_error.

    Args:
        orientation_sensor (object): orientation sensor for compass
        desired_heading (float): requested heading
        allowed_error (float): error marging for the heading
            desired_heading +- allowed_error
        
    Returns:
        float: the heading reached
    """
    heading = get_heading(orientation_sensor)
    speed = 25
    
    while abs(heading - desired_heading) > allowed_error:
        if heading - desired_heading < -allowed_error:
            print(f'Right: {heading} Req: {desired_heading} Margin: {allowed_error}')
            motor_driver.move_turn(speed, 'right', 0.2)
            
        elif heading - desired_heading > allowed_error:
            print(f'Left: {heading} Req: {desired_heading} Margin: {allowed_error}')
            motor_driver.move_turn(speed, 'right', 0.2)
            
        # if
    
        heading = get_heading(orientation_sensor)
        print(f'New heading: {heading}, delta: {heading - desired_heading}')
    
    # while
    
    print(f'Exit with heading {heading}')
    
    return heading

### correct_heading ###


def avoid_obstacle(
        motors: object,
        distance_sensor: object,
        orientation_sensor: object,
        heading: float,
        speed: float,
        safe_distance: float,
    ) -> float:
    
    # stop the motors
    motors.move_stop()
    time.sleep(0.5)
    
    # move a little bit backward
    motors.move_backward(speed)
    time.sleep(0.5)
    motors.move_stop()
    
    # turn to right untill being within a safe distance again.
    print('Turning right')
    distance = 0
    while distance < safe_distance:
        # Add 15 degrees to the heading and try to turn that way
        heading += 15 # get_heading(heading + 15)
        correct_heading(orientation_sensor, heading, 2)
        
        # motor_driver.move_turn(speed, 'right', 0.25)
        # Get the new distance for determining the safe distance
        distance = get_distance(distance_sensor)
    # while
    
    # return the new direction
    return heading

### avoid_obstacle ###
    

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

    return motor_kit, led

### initialize ###

        
def joy_ride(
        motors: object, 
        led: object,
        distance_sensor: object, 
        orientation_sensor: object, 
        speed: float, 
        safe_distance: float,
    ) -> None:
    """ Drives a robotic vehicle with simple assumptions

    Args:
        motors: instantiation of MotorKit
        led: instantiation of led
        distance_sensor: instantiation of a distance
        orientation_sensor (object): sensor showing orientation in time and space
        speed (float): speed between 0 and 100
        safe_distance (float): value below which the diustance is considered
            unsafe and requiring action
    """
    # create log file
    with open('robot.log', 'w') as logfile:
        distance = get_distance(distance_sensor)
        heading = get_heading(orientation_sensor)
        logfile.write(f'Start Speed: {speed} Distance: {distance} Heading: {heading}\n')
        
    # everything setup: motors can be started
    state: str = 'Driving'
    clock_bored = time.monotonic()
    requested_heading = 180

    try:
        while True:
            distance = get_distance(distance_sensor)
            heading = get_heading(orientation_sensor)
            correct_heading(orientation_sensor, requested_heading, 2)
            
            print(f"Status: {state} Distance: {distance} cm Heading: {heading} degrees")

            # robot has a safe distance, just cruise ahead
            if distance >= safe_distance:
                motors.move_forward(speed)
                
            # out of safe distance limit, take action
            else:
                heading = avoid_obstacle(
                    motors = motors,
                    distance_sensor = distance_sensor,
                    orientation_sensor = orientation_sensor,
                    heading = heading,
                    speed = speed, 
                    safe_distance = safe_distance,
                )
                
                print(f'*** Heading is now {heading}')
                
            # if
            
        # while
        
    except KeyboardInterrupt:
        pass

    finally:
        state = 'Final Stop'
        motors.move_stop()
        led.value = False
        print('=== Stopped ===')

    # try..finally
    
    return

### joy_ride ###


if __name__ == '__main__':
    print('=== Initializing ===')
    #################### Create I2C bus ####################
    PIN_SCL = board.GP27
    PIN_SDA = board.GP26

    # maak een I2C bus aan
    i2c = busio.I2C(scl = PIN_SCL, sda = PIN_SDA)

    #################### Get VL53L1X distance sensor ####################
    vl53 = get_vl53_sensor(i2c)
    vl53.start_ranging()
    
    #################### Get BNO055 orientation sensor ####################
    bno055 = get_bno055_sensor(i2c)
    last_val = 0xFFFF

    #################### Motors ####################
    # set constants
    MAX_SPEED: int = 20
    SAFE_DISTANCE: int = 50 # cm, value for a safe distance

    # get the most applicable chassis
    motor_board = lib_kitronik_motor.GENERIC_4WD

    # connect to the motor board
    motor_driver, led = initialize(motor_board)
    motor_driver.set_motors(False)    

    #################### Start driving ####################
    print('=== Starting a joy ride ===')
    joy_ride(
        motors = motor_driver, 
        led = led, 
        distance_sensor = vl53, 
        orientation_sensor = bno055, 
        speed = MAX_SPEED, 
        safe_distance = SAFE_DISTANCE,
    )

