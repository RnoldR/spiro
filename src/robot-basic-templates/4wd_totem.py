import sys
import time
import board
import busio
import digitalio
import adafruit_hcsr04
import adafruit_vl53l1x
import lib_kitronik_motor


def get_distance_sonar(dist_sensor: Object):
    error_count = 0
    
    while True:
        try:
            distance = dist_sensor.distance
            
            # found some distance, return it
            return distance
        
        # runtime error: probably distance too far, try again
        except RuntimeError:
            error_count += 1
            if error_count > 10:
                raise RuntimeError('Sonar detected more than 10 errors')
            
        # try..except
        
    # while

### get_distance_sonar ###
    
    
def get_distance_vl53(sensor: Object):
    sensor.start_ranging()

    sensor.start_ranging()
    while not sensor.data_ready:
        pass
            
    #sensor.clear_interrupt()
    
    distance = sensor.distance
    if distance is None:
        distance = 401
        
    return distance

### get_distance_vl53 ###


def get_distance(sensor: Object):
    if isinstance(sensor, adafruit_vl53l1x.VL53L1X):
        distance = get_distance_vl53(sensor)
        
    if isinstance(sensor, adafruit_hcsr04.HCSR04):
        distance = get_distance_sonar(sensor)
        
    return distance

### get_distance ###


def initialize(scl, sda):
    # create motor driver
    motor_kit = lib_kitronik_motor.MotorKit(lib_kitronik_motor.TOTEM)

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

        
def joy_ride(motors, led, sensor):
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

# start driving
if __name__ == '__main__':
    print('=== Starting ===')
    
    motors, led, sensor = initialize(scl = PIN_SCL, sda = PIN_SDA)
    print('=== Initialized ===')
    
    joy_ride(motors, led, sensor)
