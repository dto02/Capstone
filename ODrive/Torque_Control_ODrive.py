import odrive
from odrive.enums import MotorType, EncoderMode, AxisState, ControlMode, CurrentControlMode
import time
import matplotlib.pyplot as plt

# Initialize ODrive
print("Finding an ODrive...")
od = odrive.find_any()
print("Found ODrive!")

# Access axis and components
axis = od.axis0
mo = axis.motor
enc = axis.encoder

# Motor Configuration
print("Configuring motor...")
# Motor Configuration
mo.config.pole_pairs = 7  # 14 poles / 2 = 7 pole pairs
mo.config.resistance_calib_max_voltage = 4.0  # Voltage for resistance calibration
mo.config.motor_type = MotorType.GIMBAL  # Gimbal motor type
mo.config.current_lim = 0.8  # Max continuous current (0.8 A)
mo.config.torque_constant = 0.11  # Torque constant (Nm/A)
mo.config.calibration_current = 0.5  # Current for motor calibration (keep below max current)

# Encoder Configuration
print("Configuring encoder...")
enc.config.mode = EncoderMode.SPI  # SPI mode for AS5048A
enc.config.cpr = 16384  # 14-bit resolution (2^14 = 16384)
enc.config.bandwidth = 1000  # Encoder bandwidth (adjust as needed)

# Motor and Encoder Calibration
print("Calibrating motor and encoder...")
axis.requested_state = AxisState.MOTOR_CALIBRATION
time.sleep(10)  # Wait for motor calibration to complete
axis.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
time.sleep(10)  # Wait for encoder calibration to complete

# Set closed-loop control mode
print("Setting closed-loop control mode...")
axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
mo.config.current_control_mode = CurrentControlMode.TORQUE_CONTROL  # Torque control mode

# Data collection lists
torque_data = []
speed_data = []
voltage_data = []
current_data = []

# Function to collect data
def collect_data():
    for i in range(7000):  # Collect 100 data points
        # Set torque (current) and measure speed
        torque = (i * 0.001)*mo.config.torque_constant  # Increment torque in steps
        mo.current_setpoint = torque
        time.sleep(0.1)  # Wait for the motor to stabilize
        speed = enc.vel_estimate
        voltage = mo.vbus_voltage
        current = mo.current_control.Iq_measured

        # Store data
        torque_data.append(torque)
        speed_data.append(speed)
        voltage_data.append(voltage)
        current_data.append(current)

# Collect data
print("Collecting data...")
collect_data()

# Plot Torque vs Speed
plt.figure()
plt.plot(speed_data, torque_data, 'b-')
plt.xlabel('Speed (rad/s)')
plt.ylabel('Torque (Nm)')
plt.title('Torque vs Speed')
plt.grid(True)

# Plot Voltage vs Current
plt.figure()
plt.plot(current_data, voltage_data, 'r-')
plt.xlabel('Current (A)')
plt.ylabel('Voltage (V)')
plt.title('Voltage vs Current')
plt.grid(True)

# Show plots
plt.show()

# Clean up
print("Cleaning up...")
axis.requested_state = AxisState.IDLE