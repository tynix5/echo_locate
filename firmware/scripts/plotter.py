import matplotlib.pyplot as plt
import numpy as np
import serial
import struct
import time

#################### Configure live plot #######################
plt.ion()               # interactive mode
fig, ax = plt.subplots()

# data should be in range from x:[0,1], y:[0,1]
scat = ax.scatter([], [])
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.grid(True)
################################################################

#################### Configure serial port #####################
# open serial port
ser = serial.Serial('COM6', 115200, timeout=10000)
################################################################

while True:

    # receive 8 bytes from serial
    stream = ser.read(8)

    # wait until stream fills
    if len(stream) < 8:
        continue

    # x position is first 4 bytes, y is second 4 bytes
    x_pos = stream[:4]
    y_pos = stream[4:]

    # convert to floats
    x = struct.unpack('f', x_pos)
    y = struct.unpack('f', y_pos)

    # graph new point
    scat.set_offsets([[x, y]])
    fig.canvas.draw()
    fig.canvas.flush_events()