import time
import board
import digitalio

led = digitalio.DigitalInOut(board.GP18)
led.direction = digitalio.Direction.OUTPUT

while True:
    led.value = True
    print('On')
    time.sleep(0.5)
    
    led.value = False
    print('Off')
    time.sleep(0.5)
