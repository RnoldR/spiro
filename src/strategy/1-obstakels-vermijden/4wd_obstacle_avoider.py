import sys
import math
import time
import board
import busio
import digitalio
import adafruit_vl53l1x
import adafruit_bno055


from lib_vsfsm import vsFSM
import lib_kitronik_motor


def get_vl53_sensor(i2c_bus):
    vl53 = adafruit_vl53l1x.VL53L1X(i2c_bus)

    print("Sensor: VL53L1X")
    print("------------------------------------")

    # OPTIONAL: can set non-default values
    # Mode parameter:
    # 1: Short mode max distance is limited to 1.3 m but better ambient immunity.
    # 2: Long mode can range up to 4 m in the dark with 200 ms timing budget (Default)
    vl53.distance_mode = 2
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
    print("------------------------------------")
    print()
    
    vl53.start_ranging()
    
    return vl53

### get_vl53_sensor ###


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
    
    
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

### euler_from_quaternion ###


def get_bno055_sensor(i2c_bus):
    # fetch sensor bno055
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    sensor.mode = adafruit_bno055.NDOF_MODE

    """
    Due to electro-magnetic fields being generated from
    ferro-metals and electricity the compass will never
    be well calibrated. Calibration helps, below
    some calibration values.

    Run the bno055-calibrator on a regular basis.
    """

    #sensor.offsets_magnetometer =  (258, -418, -331)
    #sensor.offsets_gyroscope =     (0, -2, 0)
    #sensor.offsets_accelerometer = (-34, -66, -33)

    print("------------------------------------")
    print("Sensor: ", sensor.__qualname__)
    time.sleep(1)
    
    print(f"Temperature: {sensor.temperature} degrees C")
    q = sensor.quaternion
    new_euler = euler_from_quaternion(q[0], q[1], q[2], q[3])
    new_euler = [x * 57.32 for x in new_euler]
    print(f"Accelerometer (m/s^2): {sensor.acceleration}")
    #print(f"Magnetometer (microteslas): {sensor.magnetic)}")
    print(f"Gyroscope (rad/sec): {sensor.gyro}")
    print(f"Euler angle: {sensor.euler}")
    print(f"New Euler:   {new_euler}")
    print(f"Quaternion:  {sensor.quaternion}")
    print(f"Linear acceleration (m/s^2): {sensor.linear_acceleration}")
    print(f"Gravity (m/s^2): {sensor.gravity}")
    print(f"Heading in degrees: {sensor.euler[0]} ({new_euler[0]})")
    print("------------------------------------")
    print()

    return sensor

### get_bno055_sensor ###


def get_direction(compass_sensor):
    
    direction = compass_sensor.euler[0]
    deviation = min (direction - 360, direction)
    #print(f'Direction: {direction} Deviation: {deviation}')

    return direction

### get_direction ###


def scan_env(board, sensor, servo: int):
    start_degree = 25
    end_degree = 150
    step_degree = (end_degree - start_degree) / 4
    iter_degrees = [start_degree + i * step_degree for i in range(5)]
    
    board.servoWrite(servo, start_degree)
    distances = {}

    time.sleep(0.75)
    far_idx = -1
    far_dist = 0
    
    for i, degrees in enumerate(iter_degrees):
        board.servoWrite(servo, degrees)
        time.sleep(0.75) #ramp speed over 10x180ms => approx 2 seconds.
        
        dist = get_distance(sensor)
        real_degree = -int(180 * (degrees - start_degree) / (end_degree - start_degree) - 90)
        distances[i] = (real_degree, dist)
        
    # for

    board.servoWrite(servo, 90)
    
    return distances

### scan_env ####


def get_best_direction(distances: dict):
    idx = 0
    dist = distances[idx][1]
    for key, value in distances.items():
        if value[1] > dist:
            idx = key
            dist = value[1]
            
        # if
        
    # for
    
    return idx

### get_best_direction ###


def all_too_small(distances: dict, minimum: int) -> bool:
    for key, value in distances.items():
        if value[1] >= minimum:
            return False

    # for

    return True

### all_too_small ###    


def find_heading(table, heading):
    for key, value in table.items():
        if heading >= value[0] and heading <= value[1]:
            return value[2]

    # Nothing found, return None
    return None

### find_heading ###


def get_distances(
    motors: Object, 
    distance_sensor: Object, 
    ignore_forward: bool
):
    """ Get the distances at -90, -45, 0, 45, 90 degrees

    Args:
        motors (Object): Ref to motor board
        distance_sensor (Object): Ref to distance sensor
        ignore_forward (bool): When True, the forward distance will not be 
            reported by setting it to -1

    Returns:
        Dict: Dictionary containing tuples with degrees and distance for the 
            specific degrees. The key is not relevant.
    """

    directions = scan_env(motors.motor_board, distance_sensor, servo = 1)
    if ignore_forward:
        directions[2] = (0, -1)
        
    return directions

### get_distances ###

        
def compute_turn(compass_sensor, distance_sensor, motors, ignore_forward, minimum_distance):

    current_heading = get_direction(compass_sensor)
    print('Current direction is:', current_heading)
    
    directions = get_distances(motors, distance_sensor, ignore_forward)
    print('Directions:', directions)
    direction = get_best_direction(directions)
    
    print('Best scan result:', direction, directions[direction])
    dir_change = directions[direction][0]
    target_distance = directions[direction][1]
    print('Best direction is:', dir_change, 'being:', target_distance)

    target_angle = current_heading + dir_change
    if target_angle > 0: target_angle -= 360
    if target_angle < 0: target_angle += 360
    
    print('Target direction is:', target_angle)
    
    # Create rotation search tables
    right = target_angle - 180
    table = {}
    if right < 0:
        table[0] = (right + 360, 360, 'right')
        table[1] = (0, target_angle, 'right')
    else:
        table[0] = (right, target_angle, 'right')
    # if
    
    left = target_angle + 180
    if left > 360:
        table[2] = (target_angle, 360, 'left')
        table[3] = (0, left - 360, 'left')
    else:
        table[2] = (target_angle, left, 'left')
    # if
    
    return table, target_angle, target_distance

### compute_turn ###


def rotate(compass_sensor, table, inputs):
    target = inputs['Target Angle']
    current_heading = get_direction(compass_sensor)
    turn = find_heading(table, current_heading)
    delta = abs(current_heading - target)
            
    print(f'[Heading: {current_heading}, Target: {target}, Delta: {delta}, Dir: {turn}]')
    
    motors.move_turn(SLOW_SPEED, turn, 0)
        
    return abs(delta) < 1, turn

### rotate ###
        

def initialize_motors(motors: dict) -> None:
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

### initialize_motors ###


def initialize_fsm():

    # Create state machine and add states
    fsm = vsFSM()
    fsm.add_states(['Fast', 'Slow', 'Stop', 'Scan', 'Back', 'Turn', 'Halt'])

    # For each state add transitions
    fsm.add_transition('Fast', lambda inputs: inputs['Interrupt'], ['Halt'])
    fsm.add_transition('Fast', lambda inputs: inputs['Distance'] >= SP_SAFE, ['Fast'])
    fsm.add_transition('Fast', lambda inputs: inputs['Distance'] < SP_SAFE, ['Slow'])

    fsm.add_transition('Slow', lambda inputs: inputs['Interrupt'], ['Halt'])
    fsm.add_transition('Slow', lambda inputs: inputs['Distance'] >= SP_SAFE, ['Fast'])
    fsm.add_transition('Slow', lambda inputs: SP_UNSAFE <= inputs['Distance'] < SP_SAFE, ['Slow'])
    fsm.add_transition('Slow', lambda inputs: inputs['Distance'] < SP_UNSAFE, ['Stop'])

    fsm.add_transition('Stop', lambda inputs: inputs['Interrupt'], ['Halt'])
    fsm.add_transition('Stop', lambda inputs: time.time() - inputs['Timer'] < DR_STOP, ['Stop'])
    fsm.add_transition('Stop', lambda inputs: time.time() - inputs['Timer'] >= DR_STOP, ['Scan'])

    fsm.add_transition('Scan', lambda inputs: inputs['Interrupt'], ['Halt'])
    fsm.add_transition('Scan', lambda inputs: inputs['Target Distance'] < SP_SAFE, ['Back'])
    fsm.add_transition('Scan', lambda inputs: True, ['Turn'])
    
    fsm.add_transition('Back', lambda inputs: inputs['Interrupt'], ['Halt'])
    fsm.add_transition('Back', lambda inputs: time.time() - inputs['Timer'] < DR_BACK, ['Back'])
    fsm.add_transition('Back', lambda inputs: True, ['Stop'])
    
    fsm.add_transition('Turn', lambda inputs: inputs['Interrupt'], ['Halt'])
    fsm.add_transition('Turn', lambda inputs: inputs['Target Reached'], ['Slow'])
    fsm.add_transition('Turn', lambda inputs: inputs, ['Turn'])

    fsm.add_transition('Halt', lambda inputs: True, ['Halt'])

    # Add input values
    fsm.set_input('Distance', SP_SAFE)
    fsm.set_input('Target Angle', 0)
    fsm.set_input('Target Distance', 0)
    fsm.set_input('Target Reached', False)
    fsm.set_input('Timer', time.time())
    fsm.set_input('Interrupt', False)

    # set start state
    fsm.set_start_state('Fast')
    
    return fsm

### initialize_fsm ###

        
def joy_ride(motors, fsm, led, distance_sensor, compass_sensor) -> None:
    """ Drives a robotic vehicle with simple assumptions

    Args:
        motors: instantiation of MotorKit
        led: instantiation of led
        sensor: instantiation of a distance sensor
    """
    # everything setup: motors can be started
    motors.set_motors(True)
    
    # set current_state to start_state
    current_state = fsm.start_state
    print('Start state: ', current_state)
    
    # while not halted process inputs and determine states
    while current_state != 'Halt':
        try:
            led.value = False
            #time.sleep(0.25)
            
            # acquire front distance 
            distance = get_distance(distance_sensor)
            fsm.set_input('Distance', distance)
            print(f'Current state: {current_state}, distance = {distance}')
            
            # compute the new state
            if current_state is None:
                print('*** current_state is None')
            
            new_state = fsm.evaluate(current_state)
            if new_state is None:
                print('*** new_state is None')
            
            if new_state != current_state:
                print('New state:', new_state)
                
                # set Timer because some states are time limited
                fsm.set_input('Timer', time.time())
                current_state = new_state
                
            # if

            # Take action based on state of the finite state machine
            led.value = True
            if current_state == 'Fast':
                motors.move_forward(MAX_SPEED)
                
            elif current_state == 'Slow':
                motors.move_forward(SLOW_SPEED)
                
            elif current_state == 'Stop':
                motors.move_stop()
                
            elif current_state == 'Scan':
                turn_table, target_angle, target_distance = compute_turn(
                    compass_sensor = compass_sensor, 
                    distance_sensor = distance_sensor, 
                    motors = motors, 
                    ignore_forward = True, 
                    minimum_distance = SP_SAFE
                )
                fsm.set_input('Target Angle', target_angle)
                fsm.set_input('Target Distance', target_distance)
                fsm.set_input('Target Reached', False)
                
            elif current_state == 'Back':
                motors.move_backward(SLOW_SPEED)
                
            elif current_state == 'Turn':
                target_reached, turn = rotate(bno055, turn_table, fsm.inputs)
                fsm.set_input('Target Reached', target_reached)
                motors.move_turn(TURN_SPEED, turn, 0)
                
            elif current_state == 'Halt':
                led.value = False
                motors.move_stop()
                
            else:
                print(f'*** Wrong state: "{current_state}" ***')
            
            # if
            
        except KeyboardInterrupt:
            fsm.set_input('Interrupt', True)
            
        # try..except
        
    # while
        
    return

### joy_ride ###


if __name__ == '__main__':
    print('=== Initializing ===')
    print()
    
    #################### Globals constants ####################
    # Distances in cm's
    SP_SAFE = 60
    SP_UNSAFE = 30

    # Durations in seconds
    DR_STOP = 1
    DR_BACK = 0.5
    
    # Speeds
    MAX_SPEED  = 75
    SLOW_SPEED = 25
    TURN_SPEED = 40
    
    #################### Create I2C bus ####################
    PIN_SDA = board.GP14
    PIN_SCL = board.GP15

    i2c = busio.I2C(scl=PIN_SCL, sda=PIN_SDA)

    #################### Get a VL53L1X distance sensor ####################
    vl53 = get_vl53_sensor(i2c)

    #################### Get a bno055 sensor (compass) ####################
    bno055 = get_bno055_sensor(i2c)
    
    #################### Motors ####################    
    # connect to the motor board
    # get a generic four-wheel-drive model
    MOTOR_BOARD = lib_kitronik_motor.GENERIC_4WD

    # initialize driving
    motors, led = initialize_motors(MOTOR_BOARD)

    # Initialize Finite State Machine
    fsm = initialize_fsm()

    # Start a joy ride
    print()
    print('=== Starting a joy ride ===')
    
    # turn_table, target, target_dist = compute_turn(bno055, vl53, motors, True, SP_SAFE)
    # fsm.set_input('Target Angle', target)
    # fsm.set_input('Target Distance', target_dist)
    # fsm.set_input('Target Reached', False)
    
    # while not fsm.inputs['Target Reached']:
    #     target_reached = rotate(bno055, turn_table, fsm.inputs)
    #     fsm.inputs['Target Reached'] = target_reached
    #     time.sleep(1)
        
    joy_ride(motors, fsm, led, vl53, bno055)
     
    led.value = True
    
    print('* Target reached')
    while True:
        time.sleep(1)
    
