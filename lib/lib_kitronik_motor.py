import sys
import time
import PicoRobotics


GENERIC_2WD = {'motors': {'left_rear': 1,
                          'right_rear': 2,
                         },
              }

GENERIC_4WD = {'motors': {'right_front': 1,
                          'right_rear': 2,
                          'left_front': 3,
                          'left_rear': 4,
                         },
              }

GENERIC_STEER = {'motors': {'left_rear': 3,
                            'right_rear': 4,
                           },
                 'servo': {'pin': 1,
                           'right': 45,
                           'neutral': 104,
                           'left': 135,
                          },
         }


class MotorKit(object):
    """ MotorKit contains simple motoric primitives
    MOTOR_BOARD = lib_kitronik_motor.GENERIC_4WD
    Defines motoric primitives like move_forward, _backward, _turn and _stop.
    The kit can be defined in three ways:
        - 4wd, steering is accomplished by reverse rotation of right and left motors
        - 2wd, steering is accomplished by reverse rotation of right and left motor
        - steer, steering by a dedicated servo

    Each type is defined by a dictionary containing motors and, optionally,
    a servo. A template for each of the three types is provided.
    
    Static variables:
        motor_board: instantiation of the pico robotics motorboard
        directions: instructions for forward (f) and backward (r)
    """
    motor_board = PicoRobotics.KitronikPicoRobotics()
    directions = ["f","r"]

    def __init__(self,
                 motor_description: dict,
                ):
        """ Initializes the motorkit.


        Args:
            motor_description (dict): Definition of the motors and servo
                Use the generic types for examples

        Raises:
            ValueError: when the motor definition is not present an exception
                is generated
        """
        
        motors = motor_description['motors']
        if motors is None:
            raise ValueError('When using MotorKit the "motors" parameters must be specified')
        
        if 'servo' in motor_description:
            steering_wheel = motor_description['servo']
        
        else:
            steering_wheel = None
            
        # if
        
        # start with motors off, half the speed and being talkative
        self.motors_on: boolean = False
        self.max_speed: int = 50
        self.verbose: boolean = True
        
        # set parameters for the steering_wheel; this usually
        # requires experimentation
        if steering_wheel is not None:
            print(steering_wheel)
            self.servo: int = steering_wheel['pin']
            self.steer_left: int = steering_wheel['left']
            self.steer_neutral: int = steering_wheel['neutral']
            self.steer_right: int = steering_wheel['right']
            
        else:
            self.servo = None
            self.steer_left = None
            self.steer_neutral = None
            self.steer_right = None
            
        # if
            
        # find out left and right motors, these are used for steering
        # when no steering wheel is present
        self.motor_list: dict = motors
        self.motor_left: dict = {}
        self.motor_right: dict = {}
        
        for key in self.motor_list:
            if 'left' in key:
                self.motor_left[key] = self.motor_list[key]
        # for        
                    
        for key in self.motor_list:
            if 'right' in key:
                self.motor_right[key] = self.motor_list[key]
        # for        
                    
        # print('All motors:  ', self.motor_list)
        # print('Left motors: ', self.motor_left)
        # print('Right motors:', self.motor_right)

        return
    
    ### __init__ ###
    
    
    def set_motors(self, setting: boolean):
        """ enables or disables the motors

        First stops the motors, next applies setting. Setting is used
        by each move_ command. When it is False, the motors will not move.

        Args:
            setting (boolean): When False, motors are disabled.
        """
        if not setting:
            self.move_stop()
        
        self.motors_on = setting
        
        return

    ### set_motors ###


    def set_max_speed(self, max_speed: int):
        """ Sets the max_speed.

        Args:
            max_speed (int): max_speed on a scale betwee 0 to 100
        """
        self.max_speed = max_speed
        
        return

    ### set_max_speed ###


    def set_verbose(self, verbose: boolean):
        """ Sets talkativity

        Args:
            verbose (boolean): True, a lot of feedback is given.
        """
        self.verbose = verbose
        
        return

    ### set_verbose ###


    def check_speed(self, speed: int):
        """ correct if speed is outside bounds

        Args:
            speed (int): returns a speed in the interval [0..MAX_SPEED]

        Returns:
            int: corrected speed
        """
        if speed > self.max_speed:
            return self.max_speed
            
        elif speed < 0:
            return 0
        
        else:
            return speed
            
        # if
        
    ### check_speed ###


    def move_forward(self, speed: int):
        """ Moves forward with a specific speed

        Args:
            speed (int): speed to move forward
        """
        # correct if speed is outside bounds
        speed = self.check_speed(speed)
        
        # only move when motors are enabled
        if self.motors_on:
            for key in self.motor_list:
                motor = self.motor_list[key]
                self.motor_board.motorOn(motor, 'f', speed)
            
        if self.verbose:
            print(f'Move forward with speed {speed}')
            
        return

    ### move_forward ###


    def move_backward(self, speed: int):
        """ Moves backward with specific speed

        Args:
            speed (int): speed to move backward
        """
        # correct if speed is outside bounds
        speed = self.check_speed(speed)
        
        # only move when motors are enabled
        if self.motors_on:
            for key in self.motor_list:
                motor = self.motor_list[key]
                self.motor_board.motorOn(motor, 'r', speed)
        
        if self.verbose:
            print(f'Move backward with speed {speed}')
            
        return

    ### move_backward ###


    def move_turn(self, speed: int, direction: str, duration: float = 0.0):
        """ Moves specified direction for an amount of time

        When a servo is present the turn will be accomplished by using 
        the steering wheel, else by reverse rotation left and right.

        Args:
            speed (int): Speed to turn
            direction (str): Direction to turn
            duration (float, optional): Duration of turn in seconds. Defaults to 0.0.
        """
        current_max = self.max_speed
        self.max_speed = 100

        if self.verbose:
            print('move_turn', direction, speed)
            
        # When no servo use reverse rotation
        if self.servo is None:
            self.move_turn_wheels(speed, direction, duration)
            
        else:
            self.move_turn_steer(speed, direction, duration)
            
        # if
        self.max_speed = current_max
        
        return
    
    ### move_turn ###


    def move_turn_steer(self, speed: int, direction: str, duration: float):
        """ Turn by using a servo

        This requires that the vehicle is moved for some duration, else no
        turning will take place. 

        Args:
            speed (int): Speed of motors during turn
            direction (str): Direction: 'left' or 'right'
            duration (float): Duration of turn in seconds

        Raises:
            ValueError: _description_
        """
        if direction == 'right':
            self.motor_board.servoWrite(self.servo, self.steer_right)
            
        elif direction == 'left':
            self.motor_board.servoWrite(self.servo, self.steer_left)
            
        elif direction == 'straight':
            self.motor_board.servoWrite(self.servo, self.steer_neutral)

        else:
            raise ValueError('Wrong direction: should be "left", "right" or "straight".')
                
        # if
        
        # Allow a short timne for the servo to move the wheels
        time.sleep(0.1)

        # move backward with a negative speed, else forward
        if speed > 0:
            self.move_forward(speed)
            
        elif speed < 0:
            self.move_backward(-speed)
            
        # if
        
        # do nothing for a duration
        time.sleep(duration)
        
        return
    
    ### move_turn_steer ###
    
    
    def move_turn_wheels(self, speed: int, direction: str, duration: float):
        """ Turn by reverse rotation left and right wheels

        Args:
            speed (int): _description_
            direction (str): _description_
            duration (float): _description_

        Raises:
            ValueError: _description_
        """
        print('move_turn_wheels', direction, speed)
        # correct if speed is outside bounds
        speed = self.check_speed(speed)
        print('move_turn_wheels after checkspeed', direction, speed)
        
        # set the direction of left and right motors
        if direction == 'right':
            dir_right: str = 'r'
            dir_left: str = 'f'

        elif direction == 'left':
            dir_right: str = 'f'
            dir_left: str = 'r'
            
        elif direction == 'straight':
            dir_right: str = 'f'
            dir_left: str = 'f'

        else:
            raise ValueError('Wrong direction: should be "left" or "right"')
                
        # if
        
        if self.motors_on:
            # turn the right motors
            for key in self.motor_right:
                motor = self.motor_right[key]
                self.motor_board.motorOn(motor, dir_right, speed)
                
            # and the left motors
            for key in self.motor_left:
                motor = self.motor_left[key]
                self.motor_board.motorOn(motor, dir_left, speed)
            
            if self.verbose:
                print(f'Turned {direction} with speed {speed}')
                
            # if

        # if
        
        # wait a duration
        time.sleep(duration)
        
        return

    ### move_turn_wheels ###


    def move_stop(self):
        """ Stop the motors by moving with speed 0
        """
        self.move_forward(0)
        
        if self.verbose:
            print(f'Stopped')
        
        return
    ### move_stop ###

### Class: MotorKit ###
    
"""
# Perform some simple tests
if __name__ == '__main__':
    try:
        driver = MotorKit(GENERIC_4WD)
        
        driver.set_verbose(True)
        driver.set_motors(True)
        driver.set_max_speed(25)
        speed = driver.max_speed
        
        driver.move_turn(0, 'straight')

        driver.move_forward(speed)
        time.sleep(1)
        driver.move_stop()
        time.sleep(1)
        
        driver.move_backward(speed)
        time.sleep(1)
        driver.move_stop()
        time.sleep(1)
        
        driver.move_turn(speed, 'right', 1)
        driver.move_stop()
        time.sleep(1)

        driver.move_turn(speed, 'left', 1)
        driver.move_stop()
        time.sleep(1)
        
        driver.move_turn(0, 'straight')
        driver.move_stop()
        
    except Exception as e:
        print(f'Exception in test run: {e}')
        
    # try..except
        
"""
