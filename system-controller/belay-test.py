import belay

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
connect_to_ball()

state = b"0"
while True:
    input("press enter to toggle state")
    state = b"1" if state == b"0" else b"0"
    send_data(state)
    print(f"Sent state: {state}")