#!/bin/python3

import os
import sys
import serial
from time import sleep
import shutil

"""
Brief: An OS agnostic script for deploying UF2 firmware files to the RPi Pico

Details:
1. Cause reset of Pico via serial connection at 1200 baud
2. Wait for Bootloader to initialise
3. Copy of UF2 file (as spec'd by user) to Pico
"""

# FAO User: edit this as appropriate
APP_NAME = "App-StepUp"
if ("win32" in sys.platform):
    FILE_UF2_DEST = "L:\."  # windows specific
elif ("darwin" in sys.platform):
    FILE_UF2_DEST = "/Volumes/RPI-RP2"  # Mac specific
else:
    exit("Could not detect host OS!")

# Serial comms setup
# PORT = "COM4"
PORT = "/dev/tty.usbmode14201"
BAUDRATE_RESET = 1200
BAUDRATE_COMMS = 115200

# Build path for firmware hex file (UF2)
FILE_UF2_SRC_PARENT_FOLDER = os.path.join(
    ".", "build", "Application", APP_NAME)
FILE_UF2_SRC = [x for x in os.listdir(
    FILE_UF2_SRC_PARENT_FOLDER) if x.endswith(".uf2")]
if len(FILE_UF2_SRC) != 1:
    raise Exception("No obvious firmware UF2 file match")

FILE_UF2_SRC_FULL_PATH = os.path.join(
    FILE_UF2_SRC_PARENT_FOLDER, FILE_UF2_SRC[0])


print(f"UF2 source path: {FILE_UF2_SRC_FULL_PATH}")


# 1. Cause reset of Pico
try:
    ser = serial.Serial(PORT, BAUDRATE_RESET, timeout=2)
except serial.SerialException as serial_error:
    print(f"Setting Pico to Bootloader mode 🥾")
except Exception as error:
    print(f"We don't expect this error ❌: {error}")
    raise error

# 2. Give Pico bootloader time to initialise
sleep(1)

# 3. Copy UF2 to Pico hardware
print("Copying...")
shutil.copy(FILE_UF2_SRC_FULL_PATH, FILE_UF2_DEST)
print("All done 🥧")
