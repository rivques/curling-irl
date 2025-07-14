import network
import espnow
import utime

from machine import Pin, PWM
from neopixel import NeoPixel

pin = Pin(3, Pin.OUT)  
np = NeoPixel(pin, 120)  # Adjust the number of pixels as needed

# A WLAN interface must be active to send()/recv()
sta = network.WLAN(network.WLAN.IF_STA)
sta.active(True)
sta.disconnect()   # Because ESP8266 auto-connects to last Access Point

print(f"Lights receiver: Active, this board's MAC: {sta.config('mac')}") # b'\xcc\x8d\xa2\x917\n'

e = espnow.ESPNow()
e.active(True)

led = PWM(Pin(15))
motor_1a = PWM(Pin(7))
motor_1b = PWM(Pin(9))
motor_2a = PWM(Pin(11))
motor_2b = PWM(Pin(12))

value1 = 0.0
value2 = 0.0

led_last_set = utime.ticks_ms()


while True:
    host, msg = e.recv(10)
    if msg:
        raws = msg.decode('utf-8').split('|')
        if len(raws) == 0:
            continue
        np.fill((0, 0, 0))
        for raw in raws:
            pixel_raw = raw.split(',')
            if len(pixel_raw) != 4:
                continue
            index = int(pixel_raw[0])
            r = int(pixel_raw[1])
            g = int(pixel_raw[2])
            b = int(pixel_raw[3])
            if 0 <= index < len(np):
                np[index] = (r, g, b)
        np.write()
        

    # fade out LED over 5 seconds if no new message received
    led.duty_u16(max(0, 65535-(utime.ticks_ms() - led_last_set) * 65535 // 5000))
