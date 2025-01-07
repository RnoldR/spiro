import gc
import os
import time
import wifi
import board
import busio
import digitalio
import ipaddress
import ipaddress
import socketpool
import adafruit_vl53l1x

from digitalio import DigitalInOut, Direction
from adafruit_httpserver.request import HTTPRequest
from adafruit_httpserver.response import HTTPResponse
from adafruit_httpserver.methods import HTTPMethod
from adafruit_httpserver.mime_type import MIMEType

import lib_simple_server as lss
import lib_kitronik_motor 

if __name__ == '__main__':
    # create the LedController server
    server = lss.SimpleServer(os.getenv('WIFI_SSID'), os.getenv('WIFI_PASSWORD'))
    
    # create motor drive
    motor_driver = lib_kitronik_motor.MotorKit()
    motor_driver.set_max_speed(50)
    motor_driver.set_motors(False)
    
    def web_content(title: str, distance, scanned):
        """
        the HTML script, setup as an f string
        this way, can insert string variables from code.py directly
        of note, use {{ and }} if something from html *actually* needs to be in brackets
        i.e. CSS style formatting
        """
        
        html = f"""
            <h1>{title}</h1>
            <p>This is a Pico W running an HTTP server with CircuitPython.</p>
            <p>Nearest distance to the Pico W is
            <span style="color: deeppink;">{distance} cm</span></p>
            <p>Start or stop the robot from here</p>
            
            <form accept-charset="utf-8" method="POST">
                <button class="button" name="LED on" value="ON" type="submit">Enable Motors</button>
            </form></p>

            <p><form accept-charset="utf-8" method="POST">
                <button class="button" name="LED off" value="OFF" type="submit">Disable Motors</button>
            </form></p>
            
            <p><form accept-charset="utf-8" method="POST">
                <button class="button" name="Forward" value="FORWARD" type="submit">Forward</button>
            </form></p>
            
            <p><form accept-charset="utf-8" method="POST">
                <button class="button" name="Backward" value="BACKWARD" type="submit">Backward</button>
            </form></p>
            
            <p><form accept-charset="utf-8" method="POST">
                <button class="button" name="Turn_left" value="TURN_LEFT" type="submit">Turn Left</button>
            </form></p>
            
            <p><form accept-charset="utf-8" method="POST">
                <button class="button" name="Turn_right" value="TURN_RIGHT" type="submit">Turn Right</button>
            </form></p>
            
            <p><form accept-charset="utf-8" method="POST">
                <button class="button" name="Stop" value="STOP" type="submit">Stop</button>
            </form></p>
            
            <p>Scanned I2C devices: {scanned}</p>
        """
        
        return html

    ### web_page ###


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
    
    
    # define the POST message handlers
    @server.server.route("/")
    def base(request: HTTPRequest):  # pylint: disable=unused-argument
        #  serve the HTML f string with content type text/html
        with HTTPResponse(request, content_type=MIMEType.TYPE_HTML) as response:
            title = 'Pico W web server'
            http_response = server.web_page(title, web_content(title, 'Unknown', 'None'))
            print('created http response')
            response.send(http_response)
            
        # with
            
        return
            
    ### base ###


    @server.server.route("/", method=HTTPMethod.POST)
    def buttonpress(request: HTTPRequest):
        # handle button presses on the site

        #  get the raw text
        raw_text = request.raw_request.decode("utf8")
        
        #  if the led on button was pressed
        if "ON" in raw_text:
            handle_on(motor_driver)
            
        #  if the led off button was pressed
        elif "OFF" in raw_text:
            handle_off(motor_driver)
            
        elif "FORWARD" in raw_text:
            handle_forward(motor_driver)
            
        elif "BACKWARD" in raw_text:
            handle_backward(motor_driver)
            
        elif "TURN_LEFT" in raw_text:
            handle_turnleft(motor_driver)
            
        elif "TURN RIGHT" in raw_text:
            handle_turnright(motor_driver)
            
        elif "STOP" in raw_text:
            handle_stop(motor_driver)
            
        else:
            print('Illegal command:', raw_text)
            
        # if
            
        #  reload site
        with HTTPResponse(request, content_type=MIMEType.TYPE_HTML) as response:
            title = 'Pico W web server'
            http_response = server.web_page(title, web_content(title, 'Unknown', 'None'))
            response.send(http_response)
            
        return

    ### buttonpress ###


    def get_distance(dist_sensor):
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

    ### get_distance ###
        

    # onboard LED setup 
    led = digitalio.DigitalInOut(board.LED)
    led.direction = digitalio.Direction.OUTPUT
    led.value = False

    # get the i2c bus
    i2c = busio.I2C(board.GP19, board.GP18)

    led.value = True

    # use it to get the distance sensor
    vl53 = adafruit_vl53l1x.VL53L1X(i2c)
    vl53.distance_mode = 2
    vl53.timing_budget = 100

    # address to ping to determine if connection is alive
    ping_address = ipaddress.ip_address("8.8.4.4")

    # get clock value for timings
    clock = time.monotonic() #  time.monotonic() holder for server ping

    while True:
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
    
        distance = get_distance(vl53)
        direction = 180
        
        print(f"Distance: {distance} cm Direction: {direction} degrees RAM: {gc.mem_free()} KB")
    
        if distance is None:
            continue
        
        elif distance > 100:
            print('full_speed ahead')
            motor_driver.move_forward(motor_driver.max_speed)
            led.value = True
            
        elif distance > 50 and distance <= 100:
            print('move carefully')
            motor_driver.move_forward(motor_driver.max_speed / 2)
            
        elif distance > 30 and distance <= 50:
            print('stop!')
            motor_driver.move_stop()
            led.value = False
            
        else:
            print('flee')
            motor_driver.move_backward(motor_driver.max_speed)
            time.sleep(0.5)
            
        # if
        
    # while

# if
