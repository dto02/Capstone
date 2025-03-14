import odrive
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

# Set control mode to POSITION_CONTROL
odrv.axis0.controller.config.control_mode = 3  # CTRL_MODE_POSITION_CONTROL
odrv.axis0.controller.config.input_mode = 5    # INPUT_MODE_TRAP_TRAJ (Trajectory mode for smooth movement)

# Ensure motor is in closed-loop control
odrv.axis0.requested_state = 8  # AXIS_STATE_CLOSED_LOOP_CONTROL

# Move to a position (example: 10 encoder counts)
target_position = 0
odrv.axis0.controller.input_pos = target_position

time.sleep(5)  # Keep it off for 1 second

odrv.axis0.requested_state = odrive.enums.AXIS_STATE_IDLE  # Turn off motor

# Create shared memory in Windows
file_mapping = mmap.mmap(-1, SHM_SIZE, SHM_NAME)

try:
    while True:
        # Read the encoder position from ODrive
        pos = odrv.axis0.pos_estimate
        pos_degrees = pos * 360.0

        # Write the float value to shared memory
        file_mapping.seek(0)
        file_mapping.write(struct.pack('f', pos_degrees))

        time.sleep(0.001)  # Adjust as needed
except KeyboardInterrupt:
    print("Stopping...")
finally:
    file_mapping.close()
