#import odrive
import time
import struct
import mmap
import ctypes

SHM_NAME = "encoder_shm"  # Name for shared memory
SHM_SIZE = 4  # 4 bytes for a float

# Connect to ODrive
print("Connecting to ODrive...")
odrv = odrive.find_any()
print("Connected!")
i = 0

# Create shared memory in Windows
file_mapping = mmap.mmap(-1, SHM_SIZE, SHM_NAME)

try:
    while True:
        # Read the encoder position from ODrive
        pos = odrv.axis0.pos_estimate
        pos_degrees = pos * 360.0 / 8192.0
        pos_degrees = i

        # Write the float value to shared memory
        file_mapping.seek(0)
        file_mapping.write(struct.pack('f', pos_degrees))

        time.sleep(0.001)  # Adjust as needed
        i = i + 1
except KeyboardInterrupt:
    print("Stopping...")
finally:
    file_mapping.close()
