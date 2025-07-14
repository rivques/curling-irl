import belay
import time

device = belay.Device('/dev/tty.usbmodem14101')

@device.setup
def setup():
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

@device.task
def connect_to_ball():
    e.send(peer, "Starting...")
    for i in range(10):
        e.send(peer, str(i)*20, True)
    e.send(peer, b'end')

@device.task
def send_data(data):
    e.send(peer, data)


setup()
time.sleep(0.5)
connect_to_ball()

while True:
    raw = input("enter motor speeds from -1.0 to 1.0: ")
    try:
        speeds = [float(x) for x in raw.split(',')]
        if len(speeds) != 2 or not all(-1.0 <= s <= 1.0 for s in speeds):
            raise ValueError("Please enter two numbers between -1.0 and 1.0, separated by a comma.")
        data = f"{speeds[0]:.2f},{speeds[1]:.2f}".encode('utf-8')
        send_data(data)
        print(f"Sent speeds: {data.decode('utf-8')}")
    except ValueError:
        print("Invalid input, please enter a number between -1.0 and 1.0")