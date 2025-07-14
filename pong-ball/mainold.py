import network
import espnow
import utime

from machine import Pin, PWM

# A WLAN interface must be active to send()/recv()
sta = network.WLAN(network.WLAN.IF_STA)
sta.active(True)
sta.disconnect()   # Because ESP8266 auto-connects to last Access Point

print(f"Pong receiver: Active, this board's MAC: {sta.config('mac')}") # b'\xcc\x8d\xa2\x917\n'

e = espnow.ESPNow()
e.active(True)

led = PWM(Pin(15))
motor_1 = PWM(Pin(7), freq=50)
motor_2 = PWM(Pin(9), freq=50)

value1 = 0.0
value2 = 0.0

led_last_set = utime.ticks_ms()

center_value = 700_000
full_speed_deviation = 500_000

def set_pwm(value1, value2):
    global motor_1, motor_2, led, led_last_set
    led.duty_u16(65535)  # Set LED to maximum brightness
    led_last_set = utime.ticks_ms()  # Update last set time for LED
    motor1_pwm_value = center_value + int(value1 * full_speed_deviation) if abs(value1) > 0.05 else 0
    motor_1.duty_ns(motor1_pwm_value)
    motor2_pwm_value = center_value + int(value2 * full_speed_deviation) if abs(value2) > 0.05 else 0
    motor_2.duty_ns(motor2_pwm_value)
    print(f"Set PWM: Motor 1: {motor1_pwm_value}, Motor 2: {motor2_pwm_value}")

while True:
    host, msg = e.recv(10)
    if msg:
        raws = msg.decode('utf-8').split(',')
        if len(raws) != 2:
            continue
       
        value1 = float(raws[0])
        value2 = float(raws[1])
        print(f"Received values: {value1}, {value2}")
        set_pwm(value1, value2)
    # fade out LED over 5 seconds if no new message received
    led.duty_u16(max(0, 65535-(utime.ticks_ms() - led_last_set) * 65535 // 5000))
