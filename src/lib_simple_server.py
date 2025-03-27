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
from adafruit_httpserver import Server, Request, Response, Websocket, GET, POST

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
        ms = wifi.radio.ping(ipv4)
        if ms is not None:
            print(f"Ping google.com: {ms * 1000} ms") # % (wifi.radio.ping(ipv4) * 1000))
        else:
            print('*** Ping google.com yielded None: ignored.')
        
        
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

        server = Server(self.pool)

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
    def handle_on():
        led.value = True
        print('LED On')
        
        return
        
    ### handle_on ###
        

    def handle_off():
        led.value = False
        print('LED Off')
        
        return
        
    ### handle_off ###
    
    
    # create the LedController server
    server = SimpleServer(os.getenv('WIFI_SSID'), os.getenv('WIFI_PASSWORD'))

    def webpage(font_family, temperature, unit):
        html = f"""
        <!DOCTYPE html>
        <html>
        <head>
        <meta http-equiv="Content-type" content="text/html;charset=utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1">
        
        <style>
        html{{font-family: {font_family}; background-color: lightgrey;
        display:inline-block; margin: 0px auto; text-align: center;}}
          h1{{color: blue; width: 200; word-wrap: break-word; padding: 2vh; font-size: 35px;}}
          
          p{{font-size: 1.5rem; width: 200; word-wrap: break-word;}}
              .button{{font-family: {font_family};display: inline-block;
              background-color: black; border: none;
              border-radius: 4px; color: white; padding: 16px 40px;
              text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}}
          
          p.dotted {{margin: auto; width: 75%; font-size: 25px; text-align: center;}}
        </style>
        </head>
        
        <body>
        <title>SPIRO</title>
        <h1>SPIRO</h1>
        <br>
        <p>This is the SPIRO robot running on a a Pico W+.</p>
        <br>
        <p class="dotted">The current ambient temperature near the Pico W is
        <span style="color: blue;">{temperature:.0f}°{unit}</span></p><br>
        
        <h1>Control the LED on the Pico W with these buttons:</h1><br>
        
        <form accept-charset="utf-8" method="POST">
        <button class="button" name="LED ON" value="ON" type="submit">LED ON</button></a></p></form>
        
        <p><form accept-charset="utf-8" method="POST">
        <button class="button" name="LED OFF" value="OFF" type="submit">LED OFF</button></a></p></form>
        
        </body></html>
        """
        
        return html
    
    ### webpage ###


    # define the POST message handlers
    @server.server.route("/")
    def base(request: Request):  # pylint: disable=unused-argument
        print('At /')
        #  serve HTML respons to request
        response = Response(request, webpage(font_family, temperature, unit), content_type="text/html")
        
        return response
            
    ### base ###


    @server.server.route("/", POST)
    def buttonpress(request: Request):
        print('At / POST')
        # handle button presses on the site

        #  get the raw text
        raw_text = request.raw_request.decode("utf8")
        #  the HTML script
        
        #  if the led on button was pressed
        if "ON" in raw_text:
            handle_on()
            
        #  if the led off button was pressed
        if "OFF" in raw_text:
            handle_off()
            
        #  reload site
        response = Response(request, webpage(font_family, temperature, unit), content_type="text/html") 
            
        return response

    ### buttonpress ###
    

    #  onboard LED setup
    led = DigitalInOut(board.LED)
    led.direction = Direction.OUTPUT
    led.value = False
    
    # address to ping to determine if connection is alhttps:ive
    ping_address = ipaddress.ip_address("8.8.4.4")
    
    # Variables used in webpage
    font_family = 'sans-serif'
    temperature = 21.3
    unit = 'C'

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