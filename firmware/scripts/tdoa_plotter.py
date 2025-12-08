import matplotlib.pyplot as plt
import numpy as np
import serial
import struct
import time

#################### Configure live plot #######################
# coordinates of reference mics
ref_coords = np.array([[0, 0], [0, 1], [1, 1]])
ref_x, ref_y = ref_coords[:,0], ref_coords[:,1]

# initialize event point
event_coords = np.array([0.5, 0.5])

plt.ion()                           # interactive mode
fig, ax = plt.subplots()

# Plot stationary reference mics
ax.scatter(ref_x, ref_y, c='red', label='Ref')

# Plot event coordinate, keep a handle to update position
event_scatter = ax.scatter(event_coords[0], event_coords[1], c='blue', label='Event')

# data should be in range from x:[0,1], y:[0,1]
ax.set_xlim(-0.1, 1.1)
ax.set_ylim(-0.1, 1.1)
ax.grid(True)
################################################################

#################### Configure serial port #####################
# open serial port
ser = serial.Serial('COM6', 115200, timeout=0.1)
ser.reset_input_buffer()
time.sleep(0.5)
ser.reset_input_buffer()
################################################################

try:
    while True:

        # receive 8 bytes from serial
        stream = ser.read(8)

        if len(stream) == 8:

            # x position is first 4 bytes, y is second 4 bytes
            event_x = stream[:4]
            event_y = stream[4:]

            # # convert to floats
            event_coords[0] = struct.unpack('f', event_x)[0]
            event_coords[1] = struct.unpack('f', event_y)[0]

            if event_coords[0] == -1.0:
                print("Invalid data\n")
                continue

            print(f"x: {event_coords[0]}\t\ty: {event_coords[1]}")

            # graph new point
            event_scatter.set_offsets([event_coords])

        plt.pause(0.1)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    ser.close()
    plt.close(fig)