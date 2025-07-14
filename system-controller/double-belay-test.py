import belay
import time

ball = belay.Device('/dev/tty.usbmodem14101')
controller = belay.Device('/dev/tty.usbmodem14301')

@ball.setup
def ball_setup():
    import network
    import espnow

    sta = network.WLAN(network.WLAN.IF_STA)  # Or network.WLAN.IF_AP
    sta.active(True)
    sta.disconnect()

    print(f"Active, this board's MAC: {sta.config('mac')}") # b"H'\xe2F\xce\xb6"

    e = espnow.ESPNow()
    e.active(True)
    peer = b"H'\xe2F\xce\x9c"   # MAC address of peer's wifi interface
    e.add_peer(peer)      # Must add_peer() before send()

@ball.task
def connect_to_ball():
    pass
    # e.send(peer, "Starting...")
    # for i in range(10):
    #     e.send(peer, str(i)*20, True)
    # e.send(peer, b'end')

@ball.task
def send_data(data):
    e.send(peer, data)

@controller.setup
def controller_setup():
    from machine import Pin, ADC
    slider = ADC(Pin(26))

@controller.task
def read_slider_raw():
    return slider.read_u16()

def read_slider():
    raw_value = read_slider_raw()
    # Assuming the slider value is between 0 and 65535, normalize it to -1.0 to 1.0
    normalized_value = (raw_value / 32767.5) - 1.0
    # deadzone
    if abs(normalized_value) < 0.1:
        normalized_value = 0.0
    return normalized_value

def set_motor_speeds(left: float, right: float):
    """
    Send motor speeds to the device.
    
    Args:
        left (float): Speed for the left motor (-1.0 to 1.0).
        right (float): Speed for the right motor (-1.0 to 1.0).
    """
    data = f"{right:.2f},{left:.2f}".encode('utf-8')
    try:
        send_data(data)
    except Exception as e:
        print(f"Error sending data:")
        print(e)
    #print(f"Sent speeds: {data.decode('utf-8')}")

ball_setup()
controller_setup()
time.sleep(0.5)
connect_to_ball()


while True:
    slider_value = read_slider()
    set_motor_speeds(slider_value, slider_value)
    print(f"Slider value: {slider_value:.2f}")
    time.sleep(0.1)  # Adjust the sleep time as needed for your application