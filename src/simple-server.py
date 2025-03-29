import os
import math
import time
import wifi
import busio
import board
import displayio
import ipaddress
import socketpool
import terminalio
import microcontroller

import adafruit_vl53l1x
import adafruit_bno055
#import adafruit_imageload

#from adafruit_display_text import label
from digitalio import DigitalInOut, Direction
from adafruit_httpserver import Server, Request, Response, Websocket, GET, POST

import lib_simple_server as lss

# create the LedController server
server = lss.SimpleServer(os.getenv('WIFI_SSID'), os.getenv('WIFI_PASSWORD'))


def handle_on():
    led.value = True
    print('Onboard LED on')
    
    return
    
### handle_on ###
    

def handle_off():
    led.value = False
    print('Onboard LED off')
    
    return
    
### handle_off ###

 
def handle_scan():
    led.value = True
    scanned: str = ''
    print('Scanning I2C bus...')
    
    # Print out any addresses found
    devices = get_i2c_str(i2c)
    print('devices', devices)

    return devices

### handle_scan ###


def web_content(values = None, devices = None):
    value_list = ''
    device_list = ''

    if values is not None:
        maxlen = 1
        
        for key, value in values.items():
            if len(key) > maxlen:
                maxlen = len(key)
            # if
        # for
        
        maxlen += 2
        for key, value in values.items():
            spaces = maxlen - len(key)
            value_list += f'<p class="mono"; style="color: black">{key}:'
            for i in range(spaces):
                value_list += '&nbsp;'
                
            value_list += f'<span style="color: darkgreen">{str(value)}</span>'
            value_list += '</p>\n'
            
        # for
    # if
    
    if devices is not None:
        for i, device in enumerate(devices):
            device_list += f'<p class="mono">{i:2}: {str(device)}</p>\n'
        # for
    # if
    
    web_page = f"""
        <!DOCTYPE html>
        <html>
        <head>
        <meta http-equiv="Content-type" content="text/html;charset=utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
        html{{font-family: sans-serif; background-color: lightgrey;
              display:inline-block; margin: 0px auto; text-align: left;}}
              
          h1{{color: darkblue; width: 200; word-wrap: break-word; font-size: 35px; text-align: left;}}
          
          p{{font-size: 1.5rem; width: 200; word-wrap: break-word; padding 4vh}}
          p.mono {{font-family: monospace; margin: auto; font-size: 24px; text-align: left; padding 4vh}}
              
          .button{{font-family: sans-serif; display: inline-block;
                   background-color: darkblue; border: none;
                   border-radius: 4px; color: white; padding: 16px 40px;
                   text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}}
                   
          p.dotted {{margin: auto;width: 75%; font-size: 24px; text-align: center;}}
          
        </style>
        </head>
        
        <body>
        <title>SPIRO Robot</title>
        <h1>SPIRO</h1>
           
        <form accept-charset="utf-8" method="POST">
        <button class="button" name="Start" value="ON" type="submit">On</button></a></p></form>

        <p><form accept-charset="utf-8" method="POST">
        <button class="button" name="Stop" value="OFF" type="submit">Off</button></a></p></form>
        
        <h1>Status</h1>
        
        <p>{value_list}</p>
        
        
        <!--p><form accept-charset="utf-8" method="POST">
        <button class="button" name="List I2C Devices" value="I2C" type="submit">List I2C Devices</button></a></p></form-->
        
        </body>
        </html>
                
    """

    #print(web_page)
    
    return web_page

### web_content ###


def get_status_dict(distance_sensor, compass_sensor):
    status = {'Distance': get_distance(distance_sensor),
              'Heading': get_direction(compass_sensor),
              'Temperature': compass_sensor.temperature,
              'State': 'Fast'}
    
    return status

### get_status_dict ###


# define GET handler
@server.server.route("/")
def base(request: Request):  # pylint: disable=unused-argument
    print('At base')
    
    #  serve HTML respons to request
    status = get_status_dict(vl53, bno055)
    web_text = web_content(status, i2c_devices)
    response = Response(request, web_text, content_type="text/html")
    
    return response
        
### base ###


# define the POST message handlers
@server.server.route("/", POST)
def buttonpress(request: Request):
    print('At buttonpress')
    # handle button presses on the site

    #  get the raw text
    raw_text = request.raw_request.decode("utf8")
    print(raw_text)
    
    #  if the led on button was pressed
    if "ON" in raw_text:
        handle_on()
        
    #  if the led off button was pressed
    if "OFF" in raw_text:
        handle_off()
        
    #  if the scan button was pressed
    #devices = None
    #if "I2C" in raw_text:
    #    devices = i2c_devices
        
    status = get_status_dict(vl53, bno055)
    web_text = web_content(status, i2c_devices)
    
    #  reload site
    response = Response(request, web_text, content_type="text/html") 
        
    return response

### buttonpress ###


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
def get_i2c_str(i2c):
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
    sensor = adafruit_bno055.BNO055_I2C(i2c_bus)
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

    return direction

### get_direction ###


def get_i2c_devices(i2c):
    # returns a list of i2c addresses connected to i2c
    while not i2c.try_lock():
        pass

    # initialize an empty list
    devices = []
    try:
        # add devices to list
        devices = [device_address for device_address in i2c.scan()]

    finally:  # unlock the i2c bus when ctrl-c'ing out of the loop
        i2c.unlock()
        pass
    
    # try..finally
    
    return devices

### get_i2c_devices ###


def get_i2c_str(i2c):
    # return a list of i2c devices in hexadecimal string format
    devices = get_i2c_devices(i2c)
    
    devices_as_string = [hex(device) for device in devices]
    
    return devices_as_string

### get_i2c_str ###


def get_sensors(i2c_bus):
    distance_sensor = get_vl53_sensor(i2c_bus)
    compass_sensor = get_bno055_sensor(i2c_bus)
    
    return distance_sensor, compass_sensor

### get_sensor ###    


# Create I2C bus 
PIN_SDA = board.GP14
PIN_SCL = board.GP15

i2c = busio.I2C(scl=PIN_SCL, sda=PIN_SDA)

# Find connected devices
i2c_devices = get_i2c_str(i2c)

# Connect devices
vl53, bno055 = get_sensors(i2c)

#  onboard LED setup
led = DigitalInOut(board.LED)
led.direction = Direction.OUTPUT
led.value = False

print(f"Connected to WiFi: {wifi.radio.ipv4_address}")

ping_address = ipaddress.ip_address("8.8.4.4")

clock = time.monotonic()

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
        
# while

i2c.unlock()
