from machine import Pin

button = Pin(39, Pin.IN, Pin.PULL_UP)  # GPIO0 as input with pull-up
last_button_state = button.value()

print("ready for input")

while True:
    current_button_state = button.value()
    if current_button_state != last_button_state:
        print(f"Button state changed: {current_button_state}")
        last_button_state = current_button_state
    pass