import bluetooth
import pygame
import time
import socket
import numpy as np


# Set the MAC address of the Bluetooth device you want to connect to
target_address = "C0:49:EF:69:FE:82"

# Search for nearby Bluetooth devices
nearby_devices = bluetooth.discover_devices()

# Loop through the nearby devices and try to connect to the target device
for address in nearby_devices:
    if address == target_address:
        # Try to connect to the target device
        print(f"Connecting to {target_address}...")
        try:
            # Create a Bluetooth socket and connect to the target device
            sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            sock.connect((target_address, 1))

            print(f"Connection Established!")

            payload = bytes([255, 0, 127, 0, 127, 0, 250])
            sock.send(payload)
            time.sleep(5)
            payload = bytes([1, 0, 127, 0, 127, 0, 254])
            sock.send(payload)  


            # # Send some data to the target device
            # sock.send("Hello, world!")

            # # Receive some data from the target device
            # data = sock.recv(1024)
            # print(f"Received: {data}")

            # Close the socket
            sock.close()

        except:
            # Failed to connect to the target device
            print(f"Failed to connect to {target_address}")
            pass

