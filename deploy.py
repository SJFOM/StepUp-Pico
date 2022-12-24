#!/bin/python3

# OS agnostic code for deploying UF2's to Pico
# 1. Cause reset of Pico via serial connection at 1200 baud
# 2. Copy of UF2 file (as spec'd by user) to Pico

import os
import serial
from time import sleep
import shutil

# Serial comms setup
PORT = "COM4"
BAUDRATE_RESET = 1200
BAUDRATE_COMMS = 115200

# temp
APP_NAME = "App-Template"
FILE_UF2_DEST = "L:\."  # windows specific

FILE_UF2_SRC = (
    "." + os.pathsep + "App-Temp" + os.pathsep + APP_NAME + os.pathsep + "TEMPLATE.uf2"
)
print(f"file src: {FILE_UF2_SRC}")


# 1. Cause reset of Pico
ser = serial.Serial(PORT, BAUDRATE_RESET)
print(f"Serial port: ${ser.name}")
sleep(2)

# 2. Copy UF2 to Pico hardware
shutil.copyfile(FILE_UF2_SRC, FILE_UF2_DEST)


print("All done 🥧")
