# tool for calibrating a continous servo
from machine import Pin, PWM
import utime

motor = PWM(Pin(9), freq=50)  # Adjust pin number as needed
# 9: a little under 700us

# sweep through the range
for ns in range(500_000, 2_000_000, 100_000):
    motor.duty_ns(ns)
    print(f"Set duty_ns: {ns}")
    utime.sleep(0.5)
