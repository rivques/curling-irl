import os
import time
import usb.device
from usb.device.cdc import CDCInterface

cdc = CDCInterface()
cdc.init(timeout=0)  # zero timeout makes this non-blocking, suitable for os.dupterm()

# pass builtin_driver=True so that we get the built-in USB-CDC alongside,
# if it's available.
usb.device.get().init(cdc, builtin_driver=True)

print("Waiting for USB host to configure the interface...")

# wait for host enumerate as a CDC device...
while not cdc.is_open():
    time.sleep_ms(100)

# Note: This example doesn't wait for the host to access the new CDC port,
# which could be done by polling cdc.dtr, as this will block the REPL
# from resuming while this code is still executing.

print("CDC port enumerated, duplicating REPL...")