import os
import time
import wifi
import busio
import board
import displayio
import ipaddress
import socketpool
import terminalio
import microcontroller

from digitalio import DigitalInOut, Direction
from adafruit_httpserver.server import HTTPServer
from adafruit_httpserver.request import HTTPRequest
from adafruit_httpserver.response import HTTPResponse
from adafruit_httpserver.methods import HTTPMethod
from adafruit_httpserver.mime_type import MIMEType

class SimpleServer(object):
    def __init__(self, ssid: str, password: str):
        
        self.server = self.create_server(ssid, password)
        
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
        print("IP address is", wifi.radio.ipv4_address)

        #  pings Google
        ipv4 = ipaddress.ip_address("8.8.4.4")
        print("Ping google.com: %f ms" % (wifi.radio.ping(ipv4) * 1000))
        
        return pool

    ### wifi_connect ###


    def web_header(self, title: str, refresh: int = None):
        extra_meta = ''
        if refresh is not None:
            extra_meta = '<meta http-equiv="refresh" content="10">'
        
        header = f"""
            <!DOCTYPE html>
            <html>
            <head>
                {extra_meta}
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
        self.pool = self.wifi_connect(ssid, password, timeout)
        print(f"Connected to WiFi: {wifi.radio.ipv4_address}")

        server = HTTPServer(self.pool)

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

if __name__ == '__main__':
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
                <button class="button" name="LED on" value="ON" type="submit">LED on</button>
            </form>

            <p><form accept-charset="utf-8" method="POST">
                <button class="button" name="LED off" value="OFF" type="submit">LED off</button>
            </form>
            
            <p>Scanned I2C devices: {scanned}</p>
        """
        
        return html

    ### web_page ###


    def handle_on():
        led.value = True
        print('Robot started')
        
        return
        
    ### handle_on ###
        

    def handle_off():
        led.value = False
        print('Robot stopped')
        
        return
        
    ### handle_off ###
    
    
    # create the LedController server
    server = SimpleServer(os.getenv('WIFI_SSID'), os.getenv('WIFI_PASSWORD'))

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


    #  onboard LED setup
    led = DigitalInOut(board.LED)
    led.direction = Direction.OUTPUT
    led.value = False
    
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
        
    # while

# if