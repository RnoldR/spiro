import os
import sys
import time
import busio
import board
import displayio
import microcontroller

# wifi imports
import ssl
import wifi
import ipaddress
import socketpool
import terminalio

# imports for mqtt
import adafruit_minimqtt
import adafruit_requests
from adafruit_io.adafruit_io import IO_HTTP, IO_MQTT, AdafruitIO_RequestError
from digitalio import DigitalInOut, Direction
from adafruit_httpserver.server import HTTPServer

# sensor imports
import adafruit_dht
from random import randint

class SimpleMQServer(object):
    def __init__(self, ssid: str, password: str):
        
        self.io = None
        self.pool = None
        self.server = self.create_server(ssid, password)
        self.feeds = {}
        
        return
    
    ### __init __ ###
    
    
    def wifi_connect(self, ssid, password, timeout = 10):
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
        ipv4 = wifi.radio.ipv4_address
        print("IP address is", ipv4)

        #  pings oneself
        ms = wifi.radio.ping(ipv4) # * 1000
        print(f"Ping {ipv4}: {ms} ms" % ())
        
        return pool

    ### wifi_connect ###


    def create_server(self, ssid, password, timeout = 10):
        #  connect to your SSID
        self.pool = self.wifi_connect(ssid, password, timeout)
        print(f"Connected to WiFi: {wifi.radio.ipv4_address}")

        self.server = HTTPServer(self.pool)

        print("starting server..")
        
        # startup the server
        try:
            self.server.start(str(wifi.radio.ipv4_address))
            print(f"Listening on http://{wifi.radio.ipv4_address}:80")
            
        #  if the server fails to begin, restart the pico w
        except OSError:
            time.sleep(5)
            print("Could not start server, trying again..")
        
        # try..except

        return self.server

    ### create_server ###
    
    
    def create_mqtt_connection(self, username: str, key: str):
        # set up MQTT connection
        self.requests = adafruit_requests.Session(self.pool,
                                                  ssl.create_default_context())
        print('Requests:', self.requests)

        # Initialize an Adafruit IO HTTP API object
        self.io = IO_HTTP(username, key, self.requests)

        return self.io
    
    ### create_mqtt_connection ###
    
    
    def create_feed(self, feed_name: str):
        self.feeds[feed_name] = self.io.get_feed(feed_name)
        
        return
    
    ### create_feed ###
    
    
    def set_feed(self, feed_name: str, data: any):
        print(f'Setting feed {feed_name} to {data}')

        feed = self.feeds[feed_name]
        self.io.send_data(feed['key'], data)
        
        time.sleep(1)
        
        return
    
    ### set_feed ###
    
    
    def get_feed(self, feed_name: str):
        data = self.io.receive_data(feed_name)['value']
        
        return data
    
    ### get_feed ###
        

### Class: SimpleMQServer ###
    
    
def get_environment(sensor):
    temp = sensor.temperature
    humidity = sensor.humidity
    
    return temp, humidity
    
### get_environment ###

        
# Define callback methods which are called when events occur
# pylint: disable=unused-argument, redefined-outer-name
def connected(client, userdata, flags, rc):
    # This function will be called when the client is connected
    # successfully to the broker.
    print("Connected to Adafruit IO! Listening for topic changes on %s" % onoff_feed)
    # Subscribe to all changes on the onoff_feed.
    client.subscribe('on-off')


def disconnected(client, userdata, rc):
    # This method is called when the client is disconnected
    print("Disconnected from Adafruit IO!")


def message(client, topic, message):
    # This method is called when a topic the client is subscribed to
    # has a new message.
    print("New message on topic {0}: {1}".format(topic, message))
    
    return

### message ###
    
    
def create_connection():
    # create the simple server
    mq_server = SimpleMQServer(os.getenv('WIFI_SSID'), os.getenv('WIFI_PASSWORD'))

    # get an mqtt object
    aio_username = os.getenv('ADAFRUIT_IO_USERNAME')
    aio_key = os.getenv('ADAFRUIT_IO_KEY')
    io = mq_server.create_mqtt_connection(aio_username, aio_key)
    
    # Set up the callback methods 
    io.on_connect = connected
    io.ondisconnect = disconnected
    io.on_message = message
    
    return mq_server

### create_connection ###


if __name__ == '__main__':
    INTERVAL = 60 # seconds
    
    # setup temp/humidity sensor
    dht = adafruit_dht.DHT11(board.GP13)

    #  onboard LED setup
    led = DigitalInOut(board.LED)
    led.direction = Direction.OUTPUT
    led.value = False
    
    # get clock value for timings
    clock = time.monotonic() - INTERVAL
    
    # create the mw server
    mq_server = create_connection()
    
    stopped = False
    while not stopped:
        try:
            # send data into the cloud. The INTERVAL delay prevents
            # the MQTT broker to shut down when more than 30 datapoints
            # per minute are sent
            if (clock + INTERVAL) < time.monotonic():
                temp, humid = get_environment(dht)
                print('')
                mq_server.set_feed('temperature', temp)
                mq_server.set_feed('humidity', humid)

                clock = time.monotonic()
                
                # get the values of a feed as a dictionary, just get the value
                onoff = int(mq_server.get_feed('on-off'))
                speed = int(mq_server.get_feed('set-speed'))
                
                led.value = bool(onoff)
                    
            # if
            
        except KeyboardInterrupt:
            print('')
            print('\nStopping regstering sensor values')
            
            stopped = True
            
        except Exception as e:
            print("Error:\n", str(e))
            print("Resetting microcontroller in 10 seconds")
            
            time.sleep(10)
            microcontroller.reset()
            
        # try .. except
        
    # while
    
# if