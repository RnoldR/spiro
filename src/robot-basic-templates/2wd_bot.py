import os
import gc
import time
import wifi
import board
import busio
import digitalio
import ipaddress
import socketpool
import microcontroller

import adafruit_bno055
import adafruit_vl53l1x

import PicoRobotics

from digitalio import DigitalInOut, Direction
from adafruit_httpserver.server import HTTPServer
from adafruit_httpserver.request import HTTPRequest
from adafruit_httpserver.response import HTTPResponse
from adafruit_httpserver.methods import HTTPMethod
from adafruit_httpserver.mime_type import MIMEType

class SimpleServer(object):
    server = None
    def __init__(self, ssid: str, password: str):
        
        self.server = self.create_server(ssid, password)
        
        return
    
    ### __init __ ###
    
    def wifi_connect(self, ssid, password, timeout):
        print("Connecting to WiFi")

        while True:
            try:
                #  connect to your SSID
                wifi.radio.connect(ssid, password)
                break
                
            except ConnectionError as e:
                print('Connection error:', str(e))
                
            # try..except
            
        # while
                
        print("Connected to WiFi")

        pool = socketpool.SocketPool(wifi.radio)

        #  prints MAC address to REPL
        print("MAC addr:", [hex(i) for i in wifi.radio.mac_address])

        #  prints IP address to REPL
        print("IP address is", wifi.radio.ipv4_address)

        #  pings Google
        ipv4 = ipaddress.ip_address("8.8.4.4")
        print("Ping google.com: %f ms" % (wifi.radio.ping(ipv4) * 1000))
        
        return pool

    ### wifi_connect ###


    def web_header(self, title: str):
        header = f"""
            <!DOCTYPE html>
            <html>
            <head>
                <meta http-equiv="Content-type" content="text/html;charset=utf-8">
                <meta name="viewport" content="width=device-width, initial-scale=1">
                
                <style>
                  html{{font-family: tahoma,sans-serif;}}
                  p{{width: 200; word-wrap: break-word;}}
                  .button{{font-size: 24px; height: 50px; width: 200px;}}
                </style>
                <title>{title}</title>
            </head>
            <body>
          """
        
        return header

    ### web_header ###


    def web_footer(self):
        footer = f"""
            </body>
        </html>
        """
        
        return footer

    ### web_footer ###


    def web_page(self, title: str, content: str):
        
        return self.web_header(title) + content + self.web_footer()

    ### web_page###
        
     
    def create_server(self, ssid, password, timeout = 10):
        #  connect to your SSID
        pool = self.wifi_connect(ssid, password, timeout)
        print(f"Connected to WiFi: {wifi.radio.ipv4_address}")

        server = HTTPServer(pool)

        print("starting server..")
        
        # startup the server
        try:
            server.start(str(wifi.radio.ipv4_address))
            print("Listening on http://%s:80" % wifi.radio.ipv4_address)
            
        #  if the server fails to begin, restart the pico w
        except OSError:
            time.sleep(5)
            print("Could not start server, trying again..")
        
        # try..except

        return server

    ### create_server ###

### Class: SimpleServer ###


class MotorKit(object):
    MOTORS_ON: boolean = True
    MAX_SPEED: int = 100
    VERBOSE: boolean = True

    motor_board = PicoRobotics.KitronikPicoRobotics()
    directions = ["f","r"]


    def __init__(self):
        
        return
    
    ### __init__ ###
    
    
    def set_motors(self, setting: boolean):
        self.MOTORS_ON = setting
        
        self.move_stop()
        
        return

    ### set_motors ###


    def set_max_speed(self, max_speed: int):
        self.MAX_SPEED = max_speed
        
        return

    ### set_max_speed ###


    def set_verbose(self, verbose: boolean):
        self.VERBOSE = verbose
        
        return

    ### set_verbose ###


    def check_speed(self, speed: int):
        # correct if speed is outside bounds
        if speed > self.MAX_SPEED:
            return self.MAX_SPEED
            
        elif speed < 0:
            return 0
        
        else:
            return speed
            
        # if
        
    ### check_speed ###


    def move_forward(self, speed: int):
        # correct if speed is outside bounds
        speed = self.check_speed(speed)
        
        # motor 1 right motor, 2 = left motor
        if self.MOTORS_ON:
            self.motor_board.motorOn(1, 'f', speed)
            self.motor_board.motorOn(2, 'f', speed)
            
            if self.VERBOSE:
                print(f'Move forward with speed {speed}')
            
        else:
            print('Motors are off')
        
        return

    ### move_forward ###


    def move_backward(self, speed: int):
        # correct if speed is outside bounds
        speed = self.check_speed(speed)
        
        if self.MOTORS_ON:
            self.motor_board.motorOn(1, 'r', speed)
            self.motor_board.motorOn(2, 'r', speed)
        
            if self.VERBOSE:
                print(f'Move backward with speed {speed}')
            
        else:
            print('Motors are off')
        
        return

    ### move_backward ###


    def move_turn(self, speed: int, direction: str, duration: float):
        # correct if speed is outside bounds
        speed = self.check_speed(speed)
        
        if self.MOTORS_ON:
            if direction == 'right':
                self.motor_board.motorOn(1, 'r', speed)
                self.motor_board.motorOn(2, 'f', speed)
                
                if self.VERBOSE:
                    print(f'Turned {direction} with speed {speed}')
                
            elif direction == 'left':
                self.motor_board.motorOn(1, 'f', speed)
                self.motor_board.motorOn(2, 'r', speed)
                
                if self.VERBOSE:
                    print(f'Turned {direction} with speed {speed}')
                
            else:
                raise ValueError('Wrong direction: should be "left" or "right"')
            
            # if

        else:
            print('Motors are off')
        
        # if
        
        time.sleep(duration)
        
        return

    ### move_forward ###


    def move_stop(self):
        self.move_forward(0)
        
        if self.VERBOSE:
            print(f'Stopped')
        
        return
    ### move_stop ###

### Class: MotorKit ###
    

if __name__ == '__main__':
    # create the LedController server
    server = SimpleServer(os.getenv('WIFI_SSID'), os.getenv('WIFI_PASSWORD'))

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
            <p><b>Control the LED on the Pico W with these buttons:</b></p>
            
            <form accept-charset="utf-8" method="POST">
            <button class="button" name="LED on" value="ON" type="submit">LED on</button></a></p></form>

            <p><form accept-charset="utf-8" method="POST">
            <button class="button" name="LED off" value="OFF" type="submit">LED off</button></a></p></form>
            
            <p>Scanned I2C devices: {scanned}</p>
        """
        
        return html

    ### web_page ###


    def handle_on():
        led.value = True
        motors.set_motors(True)
        print('==> Motors enabled')
        
        return
        
    ### handle_on ###
        

    def handle_off():
        led.value = False
        motors.set_motors(False)
        print('==> Motors disabled')
        
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
            handle_on()
            
        #  if the led off button was pressed
        if "OFF" in raw_text:
            handle_off()
            
        #  reload site
        with HTTPResponse(request, content_type=MIMEType.TYPE_HTML) as response:
            title = 'Pico W web server'
            http_response = server.web_page(title, web_content(title, 'Unknown', 'None'))
            response.send(http_response)
            
        return

    ### buttonpress ###


    # create motor driver
    motors = MotorKit()
    motors.set_max_speed(33)
    motors.set_verbose(False)
    motors.set_motors(False)

    # allocate onboard led
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

    # get compass sensor
    compass = adafruit_bno055.BNO055_I2C(i2c)
    compass.mode = adafruit_bno055.NDOF_MODE

    # get several compass readings to acclimatize the sensor
    for i in range(100):
        heading = compass.euler[0]
        
    print('Start! Heading for {heading}')

    def collect_sensor_information(dist_sensor, compass_sensor):
        # start the distance sensor
        dist_sensor.start_ranging()

        # when there is data, read it
        while not vl53.data_ready:
            pass

        distance = dist_sensor.distance
        
        # when None is returned, usually the ob ject is too far
        # just assume the upper working limit
        if distance is None:
            distance = 400
            
        dist_sensor.clear_interrupt()
            
        direction = compass_sensor.euler[0]
            
        return distance, direction

    ### collect_sensor_information ###
        
    def process():
        # address to ping to determine if connection is alive
        ping_address = ipaddress.ip_address("8.8.4.4")

        # get clock value for timings
        clock = time.monotonic() #  time.monotonic() holder for server ping

        while True:
            ### Do network things here
            # every 30 seconds, ping server 
            if (clock + 30) < time.monotonic():
                if wifi.radio.ping(ping_address) is None:
                    print("lost connection")
                else:
                    print("connected")

                clock = time.monotonic()

            # if
            
            # poll the server for incoming/outgoing requests
            server.server.poll()
            
            ### start scanning for distance data
            
            distance, direction = collect_sensor_information(vl53, compass)
            
            print(f"Distance: {distance} cm Direction: {direction} degrees RAM: {gc.mem_free()} KB")
        
            if distance is None:
                continue
            
            elif distance > 100:
                print('full_speed ahead')
                motors.move_forward(motors.MAX_SPEED)
                led.value = True
                
            elif distance > 50 and distance <=100:
                print('move carefully')
                motors.move_forward(motors.MAX_SPEED / 2)
                
            elif distance > 30 and distance <= 50:
                print('stop!')
                motors.move_stop()
                led.value = False
                
            else:
                print('flee')
                motors.move_backward(motors.MAX_SPEED)
                time.sleep(0.2)
                
            # if
                
        # while

        return

    ### process ###


    try:
        process()
                
    finally:
        print('stopped')
        motors.move_stop()
        
    # try..except
