import espnow
import usb_cdc

e = espnow.ESPNow()
peer = espnow.Peer(b'\xcc\x8d\xa2\x92p,')
e.peers.append(peer)  # Add peer to the list

e.send(peer, "Starting...")
for i in range(10):
    e.send(peer, str(i)*20)
e.send(peer, b'end')

while True:
    data = usb_cdc.data.readline()
    if data:
        print(f"Received: {data}")
        e.send(peer, b"0" if data.strip() == "0" else b"1")