import belay
import time

device = belay.Device('/dev/tty.usbmodem14201')

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
def send_data(data):
    e.send(peer, data)


setup()


while True:
    raw = input("enter motor speeds from -1.0 to 1.0: ")
    send_data(raw.encode('utf-8'))
    print(f"Sent")