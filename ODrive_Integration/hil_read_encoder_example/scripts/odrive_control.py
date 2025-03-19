import odrive
import time
import struct
import mmap
import ctypes

# Shared memory names and sizes
TORQUE_SHM_NAME = "torque_shm"  # Shared memory for torque commands (C++ writes, Python reads)
ENCODER_SHM_NAME = "encoder_shm"  # Shared memory for encoder positions (Python writes, C++ reads)
SHM_SIZE = 4  # 4 bytes for a float

INPUT_VOLTAGE = 27.0  # Input voltage to the ODrive (27V)
MOTOR_VOLTAGE_LIMIT = 11.0  # Maximum voltage allowed for the motor (11V)
MOTOR_CURRENT_LIMIT = 0.07; #A

# Connect to ODrive
print("Connecting to ODrive...")
odrv = odrive.find_any()
print("Connected!")

# Set control mode to TORQUE_CONTROL
odrv.axis0.controller.config.control_mode = 1  # CTRL_MODE_TORQUE_CONTROL
odrv.axis0.controller.config.input_mode = 1    # INPUT_MODE_PASSTHROUGH (direct torque input)

#odrv.axis0.motor.dc_voltage_limit = MOTOR_VOLTAGE_LIMIT

# Ensure motor is in closed-loop control
odrv.axis0.requested_state = 8  # AXIS_STATE_CLOSED_LOOP_CONTROL

odrv.axis0.pos_estimate = 0

# Create shared memory for torque input (C++ writes, Python reads)
try:
    torque_shm = mmap.mmap(-1, SHM_SIZE, TORQUE_SHM_NAME)
except FileNotFoundError:
    print(f"Shared memory '{TORQUE_SHM_NAME}' not found. Ensure the C++ program creates it first.")
    exit(1)

# Create shared memory for encoder output (Python writes, C++ reads)
try:
    encoder_shm = mmap.mmap(-1, SHM_SIZE, ENCODER_SHM_NAME)
except FileNotFoundError:
    print(f"Shared memory '{ENCODER_SHM_NAME}' not found. Ensure the C++ program creates it first.")
    exit(1)

try:
    while True:
        # Read the torque command from shared memory (C++ writes)
        torque_shm.seek(0)
        torque_command_bytes = torque_shm.read(SHM_SIZE)
        torque_command = struct.unpack('f', torque_command_bytes)[0]  # Unpack float

        # Apply the torque command to the motor
        odrv.axis0.controller.input_torque = torque_command

        # Read the encoder position from ODrive
        pos = odrv.axis0.pos_estimate
        pos_degrees = pos * 360.0  # Convert to degrees (optional)

        # Write the encoder position to shared memory (Python writes)
        encoder_shm.seek(0)
        encoder_shm.write(struct.pack('f', pos_degrees))

        # Print for debugging
        #print(f"Torque Command: {torque_command}, Position: {pos_degrees} degrees")

        time.sleep(0.001)  # Adjust as needed
except KeyboardInterrupt:
    print("Stopping...")
finally:
    # Cleanup
    odrv.axis0.requested_state = odrive.enums.AXIS_STATE_IDLE  # Set motor to idle mode
    torque_shm.close()
    encoder_shm.close()