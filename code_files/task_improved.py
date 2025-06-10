import time
import numpy as np
import signal
import sys
import select
import os
import datetime
import csv
import matplotlib.pyplot as plt
import platform

# Detect if we're running on a Raspberry Pi
def is_raspberry_pi():
    """Check if the code is running on a Raspberry Pi"""
    try:
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read()
            return 'Raspberry Pi' in model
    except:
        return False

# Set a global flag for platform
RUNNING_ON_PI = is_raspberry_pi()

# Import hardware-specific libraries only on Raspberry Pi
if RUNNING_ON_PI:
    import pigpio
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    import adafruit_mcp4725


# ===== Configuration Parameters =====
# Servo and Gear Configuration
SERVO_PIN = 17                # GPIO pin for servo control
MIN_PULSE_WIDTH = 500         # Pulse width in microseconds for minimum position
MAX_PULSE_WIDTH = 2500        # Pulse width in microseconds for maximum position
MAX_GEAR_ROTATION = 4         # Maximum rotation of the GEAR from center in either direction (degrees)
GEAR_RATIO = 5                # Gear ratio (1:5) - torque multiplier, rotation divider
# The servo motor will rotate MAX_GEAR_ROTATION * GEAR_RATIO degrees
# Calculate the servo motor rotation based on gear rotation
MAX_SERVO_ROTATION = MAX_GEAR_ROTATION * GEAR_RATIO  # Maximum rotation of the SERVO MOTOR

# GPIO pin for the green LED (reward)
GREEN_LED_PIN = 27

# ADC Configuration
ADS1115_CHANNEL = 0           # Channel on the ADS1115
BUFFER_ZONE = 0.0             # Buffer zone for analog input variability (in volts)

# ===== Task Parameters =====
# Disturbance Configuration
DISTURBANCE_STYLE = "sin"     # Options: "sin", "none"
DISTURBANCE_MAGNITUDE = 2    # Amplitude of disturbance in GEAR degrees (actual output rotation)
DISTURBANCE_PERIOD = 10       # Period of disturbance in seconds (time for one complete cycle)

# Reward Configuration
DELIVERY_ZONE = 1           # Size of reward zone in GEAR degrees (±5° at gear output)
TIME_FOR_REWARD = 0.5         # Time required in zone to trigger reward (seconds)

# Trial Configuration
TRIAL_DURATION = 3600         # Duration of trial in seconds (default: 1 hour)
LEVER_SENSITIVITY = 1.0       # Sensitivity of lever control (1.0 = normal)

# Plotting Configuration
PLOT_CONFIG_ONLY = True       # If True, only show configuration plot and exit (on non-Pi devices)
PLOT_ON_PI = False            # Whether to show plots on Raspberry Pi (usually False to save resources)

# ADS1115 Channel for the green sensor and lever
ADS1115_CHANNEL = 0           # Channel offset on the ADS1115

def angle_to_pulse_width(pi, angle):
    # Ensure angle is within valid range
    angle = max(0, min(180, angle))

    # Convert angle to pulse width
    pulse_width = MIN_PULSE_WIDTH + (angle / 180) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)

def diagnose_i2c():
    """
    Run diagnostics on the I2C bus and print helpful information
    """
    print("\n=== I2C Diagnostics ===")

    try:
        # Try to detect I2C devices using the i2cdetect command
        print("Detecting I2C devices...")
        result = os.popen("i2cdetect -y 1").read()
        print(result)

        # Check if the ADS1115 is detected (address 0x48)
        if "48" in result:
            print("ADS1115 detected at address 0x48 ✓")
        else:
            print("WARNING: ADS1115 not detected! Check connections.")

        # Print some helpful advice
        print("\nIf you're experiencing I2C errors, try the following:")
        print("1. Check all connections and ensure wires are secure")
        print("2. Use shorter, higher quality wires")
        print("3. Add a capacitor (100nF) between VCC and GND near the ADS1115")
        print("4. Move wires away from sources of interference (motors, power supplies)")
        print("5. Use a separate power supply for the servo motor")
        print("6. Add pull-up resistors (4.7kΩ) to SDA and SCL lines")
        print("7. Reduce the servo movement speed or range")

    except Exception as e:
        print(f"Error running diagnostics: {e}")

    print("=== End of Diagnostics ===\n")

    # Ask if user wants to continue
    print("Do you want to continue with the program? (y/n)")
    response = input().strip().lower()
    return response == 'y' or response == 'yes'

def get_sensor_voltage(sensor, num_samples=20, max_retries=5):
    """
    Get averaged sensor voltage with outlier removal and error handling
    """
    readings = []
    retry_count = 0

    while len(readings) < num_samples:
        try:
            # Try to get a voltage reading
            voltage = sensor.voltage  # Use voltage instead of raw value
            readings.append(voltage)
            print(".", end="", flush=True)
        except OSError as e:
            # Handle I2C errors
            retry_count += 1
            if retry_count > max_retries:
                print(f"\nToo many I2C errors ({retry_count}). Using available readings.")
                break
            print("x", end="", flush=True)  # Show error with 'x'
            time.sleep(0.1)  # Longer delay after an error

        time.sleep(0.05)

    # If we have no readings, return a default value
    if not readings:
        print("\nWarning: No valid readings obtained. Using default value.")
        return 2.5  # Default mid-range voltage

    # Sort and trim outliers if we have enough readings
    readings.sort()
    if len(readings) > 10:  # Only trim if we have enough readings
        trimmed_readings = readings[5:-5]  # Remove 5 lowest and 5 highest values
    else:
        trimmed_readings = readings  # Use all readings if we don't have many

    return sum(trimmed_readings) / len(trimmed_readings)

def dispense_reward(pi, duration=0.007):
    """
    Flash the green LED for the specified duration
    """
    pi.write(GREEN_LED_PIN, 1)  # Turn on LED
    time.sleep(duration)
    pi.write(GREEN_LED_PIN, 0)  # Turn off LED

def voltage_to_degrees(voltage, resting_voltage, max_voltage, min_angle, max_angle):
    """
    Convert a voltage to degrees, works regardless of whether
    max_voltage is greater than or less than resting_voltage.
    """
    # Determine if voltage increases or decreases as lever rotates
    if max_voltage > resting_voltage:  # Voltage increases with rotation
        # If voltage is at or below resting, return min_angle
        if voltage <= resting_voltage:
            return float(min_angle)

        # If voltage is at or above max, return max_angle
        if voltage >= max_voltage:
            return float(max_angle)

        # Calculate degrees based on how far the voltage is from resting toward max_voltage
        return min_angle + ((voltage - resting_voltage) / (max_voltage - resting_voltage)) * (max_angle - min_angle)

    else:  # Voltage decreases with rotation (max_voltage < resting_voltage)
        # If voltage is at or above resting, return min_angle
        if voltage >= resting_voltage:
            return float(min_angle)

        # If voltage is at or below max, return max_angle
        if voltage <= max_voltage:
            return float(max_angle)

        # Calculate degrees based on how far the voltage is from resting toward max_voltage
        return min_angle + ((resting_voltage - voltage) / (resting_voltage - max_voltage)) * (max_angle - min_angle)

def get_disturbance(disturbance_style, current_time, servo_center=90, magnitude=None, period=None):
    """
    Returns the angle that the disturbance should be at given the current time

    The disturbance is centered at servo_center and oscillates based on the provided parameters.

    Note: magnitude is in GEAR degrees and will be converted to servo degrees internally
    """
    # Use default values if not provided
    if magnitude is None:
        magnitude = MAX_GEAR_ROTATION

    if period is None:
        period = 2 * np.pi  # Default to 2π seconds (1 Hz)

    if disturbance_style == "sin":
        # Calculate frequency from period (f = 1/T)
        frequency = 1.0 / period

        # Convert magnitude from gear degrees to servo degrees
        servo_magnitude = magnitude * GEAR_RATIO

        # This will oscillate between servo_center-servo_magnitude and servo_center+servo_magnitude
        # with the specified period
        return np.sin(2 * np.pi * frequency * current_time) * servo_magnitude + servo_center
    elif disturbance_style == "none":
        # No disturbance - return servo_center
        return servo_center
    else:
        # Default to no disturbance if style is unknown
        print(f"Unknown disturbance style: {disturbance_style}. Using 'none'.")
        return servo_center

def determine_servo_center(pi, servo_sensor):
    """
    Interactive function to determine the servo center position
    """
    print("\n=== Servo Center Determination ===")
    print("This will help determine the center position of the servo.")
    print("The current angle is 90 degrees (default center).")
    print("Enter a new angle to move the servo, or 'f' when the servo is centered.")

    current_angle = 90.0
    angle_to_pulse_width(pi, current_angle)
    time.sleep(1)  # Give servo time to move

    while True:
        # Read current sensor value
        servo_voltage = servo_sensor.voltage
        print(f"\nCurrent angle: {current_angle:.1f}° | Sensor reading: {servo_voltage:.3f}V")

        # Get user input
        user_input = input("Enter new angle (0-180) or 'f' to finish: ").strip().lower()

        if user_input == 'f':
            print(f"Servo center set to {current_angle:.1f} degrees")
            return current_angle

        try:
            new_angle = float(user_input)
            if 0 <= new_angle <= 180:
                current_angle = new_angle
                angle_to_pulse_width(pi, current_angle)
                time.sleep(0.5)  # Give servo time to move
            else:
                print("Angle must be between 0 and 180 degrees")
        except ValueError:
            print("Invalid input. Enter a number between 0 and 180, or 'f' to finish")

def save_calibration(servo_center, green_min, green_max, lever_min, lever_max):
    """
    Save calibration data to a file
    """
    # Create calibration directory if it doesn't exist
    calib_dir = os.path.join(os.getcwd(), "calibration_data")
    if not os.path.exists(calib_dir):
        os.makedirs(calib_dir)

    # Create calibration file
    calib_file = os.path.join(calib_dir, "task_calibration.csv")
    with open(calib_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["servo_center", "green_min", "green_max", "lever_min", "lever_max"])
        writer.writerow([f"{servo_center:.6f}", f"{green_min:.6f}", f"{green_max:.6f}",
                         f"{lever_min:.6f}", f"{lever_max:.6f}"])

    print(f"Calibration data saved to {calib_file}")

def load_calibration():
    """
    Load calibration data from a file
    """
    calib_file = os.path.join(os.getcwd(), "calibration_data", "task_calibration.csv")
    if not os.path.exists(calib_file):
        return None

    try:
        with open(calib_file, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # Skip header
            row = next(reader)
            return float(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4])
    except (IndexError, ValueError, StopIteration):
        print("Error reading calibration file. Will perform new calibration.")
        return None

# This function is no longer needed as we'll use mice ID instead of trial numbers
# Keeping it as a placeholder to avoid changing too much code structure
def get_trial_number():
    return 1

# Global variables
servo_center_global = 90  # Store servo center position
plot_fig = None

def create_configuration_plot(servo_center, delivery_zone, max_gear_rotation, disturbance_style,
                             disturbance_magnitude=None, disturbance_period=None, gear_ratio=GEAR_RATIO):
    """
    Create a static plot showing the configuration parameters
    """
    # Calculate the corresponding servo rotation
    max_servo_rotation = max_gear_rotation * gear_ratio

    # For backward compatibility with the rest of the function
    max_rotation = max_servo_rotation
    global plot_fig

    # Create figure and subplots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
    fig.tight_layout(pad=3.0)
    fig.suptitle('Task Configuration', fontsize=16)

    # Subplot 1: Lever vs Gear Position
    ax1.set_title('Lever vs Gear Position')
    ax1.set_xlabel('Lever Angle (degrees)')
    ax1.set_ylabel('Gear Angle (degrees relative to center)')
    ax1.grid(True)

    # Plot the lever-to-gear mapping
    lever_angles = np.linspace(0, 45, 100)
    gear_angles = []

    for lever_angle in lever_angles:
        # Calculate the servo angle based on lever position
        servo_offset = -max_rotation + (2 * max_rotation * lever_angle / 45)
        # Convert to gear angle (divide by gear ratio)
        gear_angle = servo_offset / gear_ratio
        gear_angles.append(gear_angle)

    ax1.plot(lever_angles, gear_angles, 'b-', label='Lever-to-Gear Mapping')

    # Add reward zone (already in gear angles)
    ax1.axhspan(-delivery_zone, delivery_zone,
               alpha=0.2, color='green', label='Reward Zone')
    ax1.text(40, 0, 'Reward Zone', color='green')

    # Add safe range (in gear angles)
    ax1.axhspan(-max_gear_rotation, max_gear_rotation,
               alpha=0.1, color='blue', label='Safe Range')

    # Set axis limits
    ax1.set_xlim(0, 45)
    ax1.set_ylim(-max_gear_rotation - 1, max_gear_rotation + 1)
    ax1.legend(loc='upper right')

    # Subplot 2: Disturbance over Time
    ax2.set_title('Disturbance Pattern')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Gear Disturbance Angle (degrees relative to center)')
    ax2.grid(True)

    # Plot the disturbance pattern
    if disturbance_style == "sin" and disturbance_magnitude is not None and disturbance_period is not None:
        time_values = np.linspace(0, disturbance_period * 2, 200)
        disturbance_values = []

        for t in time_values:
            # Calculate disturbance using the same formula as in the main code
            frequency = 1.0 / disturbance_period
            # Calculate disturbance directly in gear degrees (disturbance_magnitude is already in gear degrees)
            gear_disturbance = np.sin(2 * np.pi * frequency * t) * disturbance_magnitude
            disturbance_values.append(gear_disturbance)

        ax2.plot(time_values, disturbance_values, 'r-', label='Sinusoidal Disturbance')

        # Add reward zone to the middle graph (already in gear angles)
        ax2.axhspan(-delivery_zone, delivery_zone,
                   alpha=0.2, color='green', label=f'Reward Zone (±{delivery_zone:.1f}°)')

        # Add safe range (in gear angles)
        ax2.axhspan(-max_gear_rotation, max_gear_rotation,
                   alpha=0.1, color='blue', label='Safe Range')

        ax2.set_xlim(0, disturbance_period * 2)
        ax2.set_ylim(-disturbance_magnitude - 1, disturbance_magnitude + 1)
        ax2.legend(loc='upper right')
    else:
        # Add reward zone even for no disturbance (already in gear angles)
        ax2.axhspan(-delivery_zone, delivery_zone,
                   alpha=0.2, color='green', label=f'Reward Zone (±{delivery_zone:.1f}°)')

        # Add safe range (in gear angles)
        ax2.axhspan(-max_gear_rotation, max_gear_rotation,
                   alpha=0.1, color='blue', label='Safe Range')

        ax2.text(0.5, 0.5, 'No Disturbance', horizontalalignment='center',
                verticalalignment='center', transform=ax2.transAxes, fontsize=14)

        # Set reasonable y-limits to show the reward zone
        ax2.set_ylim(-max_gear_rotation - 1, max_gear_rotation + 1)
        ax2.legend(loc='upper right')

    # Subplot 3: Combined visualization (stacking the first two graphs)
    ax3.set_title('Combined Visualization')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Gear Angle (degrees relative to center)')
    ax3.grid(True)

    # Create a sample visualization that stacks the first two graphs
    time_values = np.linspace(0, disturbance_period * 2 if disturbance_period else 10, 200)

    # Create a straight diagonal line for lever contribution
    # Start at 0 degrees and end at 45 degrees
    lever_angles = np.linspace(0, 45, len(time_values))

    # Convert lever angles to gear offsets
    gear_offsets = []
    for angle in lever_angles:
        # Calculate servo offset
        servo_offset = -max_rotation + (2 * max_rotation * angle / 45)
        # Convert to gear offset
        gear_offset = servo_offset / gear_ratio
        gear_offsets.append(gear_offset)

    # Get gear disturbance values
    gear_disturbance_values = []
    if disturbance_style == "sin" and disturbance_magnitude is not None and disturbance_period is not None:
        for t in time_values:
            frequency = 1.0 / disturbance_period
            # Calculate disturbance directly in gear degrees (disturbance_magnitude is already in gear degrees)
            gear_disturbance = np.sin(2 * np.pi * frequency * t) * disturbance_magnitude
            gear_disturbance_values.append(gear_disturbance)
    else:
        # No disturbance, all zeros
        gear_disturbance_values = [0] * len(time_values)

    # Calculate final gear angles (sum of gear offset from lever and gear disturbance)
    final_gear_values = []
    for i in range(len(time_values)):
        final = gear_offsets[i] + gear_disturbance_values[i]
        # Ensure within safe range
        final = max(-max_gear_rotation, min(max_gear_rotation, final))
        final_gear_values.append(final)

    # Plot all values
    ax3.plot(time_values, gear_offsets, 'b-', label='Lever Contribution')

    if disturbance_style == "sin":
        ax3.plot(time_values, gear_disturbance_values, 'r-', label='Disturbance')

    ax3.plot(time_values, final_gear_values, 'k-', label='Final Gear Position')

    # Add reward zone (already in gear angles)
    ax3.axhspan(-delivery_zone, delivery_zone,
               alpha=0.2, color='green', label=f'Reward Zone (±{delivery_zone:.1f}°)')

    # Add safe range (in gear angles)
    ax3.axhspan(-max_gear_rotation, max_gear_rotation,
               alpha=0.1, color='blue', label='Safe Range')

    # Set axis limits
    ax3.set_xlim(0, disturbance_period * 2 if disturbance_period else 10)
    ax3.set_ylim(-max_gear_rotation - 1, max_gear_rotation + 1)
    ax3.legend(loc='upper right')

    # Add configuration information as text
    config_text = f"Configuration:\n"
    config_text += f"Servo Center: {servo_center:.1f}°\n"

    # Calculate servo delivery zone from gear delivery zone
    servo_delivery_zone = delivery_zone * gear_ratio

    config_text += f"Reward Zone: ±{delivery_zone:.1f}° (gear) / ±{servo_delivery_zone:.1f}° (servo)\n"
    config_text += f"Max Gear Rotation: {max_gear_rotation:.1f}°\n"
    config_text += f"Gear Ratio: 1:{gear_ratio}\n"
    config_text += f"Servo Motor Rotation: {max_servo_rotation:.1f}°\n"
    config_text += f"Disturbance: {disturbance_style}\n"

    if disturbance_style == "sin":
        # Show disturbance magnitude in both gear and servo degrees
        servo_disturbance_magnitude = disturbance_magnitude * gear_ratio
        config_text += f"Magnitude: {disturbance_magnitude:.1f}° (gear) / {servo_disturbance_magnitude:.1f}° (servo)\n"
        config_text += f"Period: {disturbance_period}s"

    fig.text(0.02, 0.02, config_text, fontsize=10,
             bbox=dict(facecolor='white', alpha=0.8))

    # Store figure in global variable
    plot_fig = fig

    return fig

# is_raspberry_pi function is defined at the top of the file

def signal_handler(sig, frame):  # pylint: disable=unused-argument
    """
    Handle Ctrl+C by centering the servo before exiting
    """
    print("\nCentering servo and exiting...")
    pi = pigpio.pi()
    if pi.connected:
        # Use the global servo center position
        angle_to_pulse_width(pi, servo_center_global)
        time.sleep(0.5)  # Give servo time to move
        pi.stop()

    # Close plot if it exists
    if plot_fig is not None:
        plt.close(plot_fig)

    sys.exit(0)

def main():
    # Declare global variables at the beginning of the function
    global servo_center_global, dac_available

    # Initialize dac_available flag
    dac_available = False

    # If we're not on a Raspberry Pi, just show the configuration plot and exit
    if not RUNNING_ON_PI:
        print("Running on a non-Raspberry Pi device. Showing configuration plot...")

        # Use default values for the plot
        servo_center = 90
        delivery_zone = DELIVERY_ZONE

        # Create the configuration plot with default values
        create_configuration_plot(
            servo_center,
            delivery_zone,
            MAX_GEAR_ROTATION,  # Using gear rotation as the primary parameter
            "sin",  # Default to sine wave disturbance
            DISTURBANCE_MAGNITUDE,  # This is in servo degrees, will be converted in the function
            DISTURBANCE_PERIOD,
            GEAR_RATIO
        )

        plt.show()  # This will block until the plot window is closed
        print("Configuration plot closed. Exiting.")
        return

    # The following code only runs on Raspberry Pi

    # Set up signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # Initialize pigpio
    pi = pigpio.pi()
    if not pi.connected:
        print("Failed to connect to pigpio daemon")
        exit(1)

    # Set up the green LED pin as output
    pi.set_mode(GREEN_LED_PIN, pigpio.OUTPUT)
    pi.write(GREEN_LED_PIN, 0)  # Ensure LED is off at start

    # Run I2C diagnostics first
    print("\nWould you like to run I2C diagnostics? (recommended if you're having I2C errors) (y/n)")
    run_diagnostics = input().strip().lower() == 'y'

    if run_diagnostics:
        if not diagnose_i2c():
            print("Exiting program as requested.")
            exit(0)

    # Initialize I2C bus, ADS1115, and MCP4725 with error handling
    try:
        # Set up I2C with a lower frequency to improve reliability
        i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)  # 100 kHz instead of default 400 kHz

        # Add a small delay to ensure I2C bus is ready
        time.sleep(0.5)

        # Initialize ADS1115 ADC
        ads = ADS.ADS1115(i2c)
        ads.gain = 1  # Set gain for 5V sensor

        # Set data rate to lowest setting for more reliable readings
        # This is a direct register write to set the data rate to 8 SPS (samples per second)
        # The default is 128 SPS, which can be too fast and cause errors
        ads._write_register(0x01, 0x8583)  # Config register with data rate bits set to 000 (8 SPS)

        # Initialize MCP4725 DAC
        try:
            # Default address for MCP4725 is 0x62
            dac = adafruit_mcp4725.MCP4725(i2c, address=0x60)
            print("MCP4725 DAC initialized successfully")
            dac_available = True
        except Exception as dac_error:
            print(f"Warning: Could not initialize MCP4725 DAC: {dac_error}")
            print("The program will continue without DAC output.")
            dac_available = False

        # Add another small delay after configuration
        time.sleep(0.5)

        servo_sensor = AnalogIn(ads, ADS.P1 + ADS1115_CHANNEL)  # Green sensor reads servo position
        lever_sensor = AnalogIn(ads, ADS.P0 + ADS1115_CHANNEL)  # Hall sensor reads lever position

        print("I2C and ADC initialized successfully")
    except Exception as e:
        print(f"Error initializing I2C: {e}")
        print("Please check your connections and try again.")
        print("You might want to run the program again with diagnostics enabled.")
        exit(1)

    # Function to update DAC output based on disturbance
    def update_dac_output(angle, min_angle=0, max_angle=180):
        """
        Update the MCP4725 DAC output based on the given angle.
        Maps the angle to a 12-bit value (0-4095) for the DAC.
        """
        global dac_available
        if not dac_available:
            return

        try:
            # Map the angle to a 12-bit value (0-4095)
            # When angle is min_angle, DAC outputs 0V
            # When angle is max_angle, DAC outputs ~3.3V
            normalized = (angle - min_angle) / (max_angle - min_angle)
            dac_value = int(normalized * 4095)
            dac_value = max(0, min(4095, dac_value))  # Ensure value is in range

            # Update the DAC output
            dac.raw_value = dac_value
        except Exception as dac_error:
            # If there's an error, disable the DAC for future calls
            print(f"\nError updating DAC: {dac_error}")
            dac_available = False

    # Create data directory if it doesn't exist
    data_dir = os.path.join(os.getcwd(), "task_data")
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)

    # Ask for mice ID
    print("\nPlease enter the mice ID (e.g., D147M51):")
    mice_id = input().strip()

    # Validate mice ID (basic validation)
    while not mice_id:
        print("Mice ID cannot be empty. Please enter a valid mice ID:")
        mice_id = input().strip()

    # Create data file with mice ID
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    data_filename = os.path.join(data_dir, f"{mice_id}_{timestamp}.txt")

    print(f"Data will be saved to: {data_filename}")

    # Create and open data file
    with open(data_filename, 'w', newline='') as data_file:
        # Write header
        data_writer = csv.writer(data_file, delimiter='\t')
        data_writer.writerow([
            "Time (s)",
            "Servo Angle (degrees)",
            "Lever Angle (degrees)",
            "Disturbance Angle",
            "Final Angle",
            "Reward Delivered (0/1)"
        ])

        # Check if previous calibration exists
        calibration_data = load_calibration()
        use_previous_calibration = False

        if calibration_data:
            print("Previous calibration data found.")
            print("Do you want to use the previous calibration? (y/n)")
            response = input().strip().lower()
            use_previous_calibration = response == 'y' or response == 'yes'

        if use_previous_calibration:
            servo_center, servo_min, servo_max, lever_rest, lever_max = calibration_data
            print(f"Using previous calibration:")
            print(f"Servo center: {servo_center:.1f} degrees")
            print(f"Servo sensor range: {servo_min:.3f}V to {servo_max:.3f}V")
            print(f"Lever sensor range: {lever_rest:.3f}V (rest) to {lever_max:.3f}V (extended)")

            # Update global variable for signal handler
            servo_center_global = servo_center

            # When using previous calibration, we need to set servo_center_voltage
            # We'll use the average of servo_min and servo_max as an approximation
            servo_center_voltage = (servo_min + servo_max) / 2

            # Also set servo_left and servo_right for angle calculation
            servo_left = servo_min
            servo_right = servo_max

            # Detect and inform about lever direction from previous calibration
            if lever_max > lever_rest:
                print("Detected: Lever voltage INCREASES when extended")
            else:
                print("Detected: Lever voltage DECREASES when extended")
        else:
            print("Starting calibration sequence...")

            # First, determine the servo center
            servo_center = determine_servo_center(pi, servo_sensor)
            # Update global variable for signal handler
            servo_center_global = servo_center

            # Now calibrate the servo sensor at different positions
            print("\nCalibrating servo sensor...")

            # Move servo to left position (servo_center - MAX_SERVO_ROTATION degrees)
            left_angle = max(0, servo_center - MAX_SERVO_ROTATION)
            print(f"\nMoving to left position ({left_angle:.1f} degrees)...")
            print(f"This is {MAX_SERVO_ROTATION} degrees left of center (servo)")
            print(f"This corresponds to {MAX_GEAR_ROTATION} degrees of gear rotation")
            angle_to_pulse_width(pi, left_angle)
            time.sleep(2)  # Wait for servo to settle

            # Get servo sensor reading
            print("Taking servo sensor reading...")
            servo_left = get_sensor_voltage(servo_sensor)
            print(f"Left position: Servo={servo_left:.3f}V")

            # Move servo to center position
            print(f"\nMoving to center position ({servo_center:.1f} degrees)...")
            angle_to_pulse_width(pi, servo_center)
            time.sleep(2)

            # Get servo sensor reading
            print("Taking servo sensor reading...")
            servo_center_voltage = get_sensor_voltage(servo_sensor)
            print(f"Center position: Servo={servo_center_voltage:.3f}V")

            # Move servo to right position (servo_center + MAX_SERVO_ROTATION degrees)
            right_angle = min(180, servo_center + MAX_SERVO_ROTATION)
            print(f"\nMoving to right position ({right_angle:.1f} degrees)...")
            print(f"This is {MAX_SERVO_ROTATION} degrees right of center (servo)")
            print(f"This corresponds to {MAX_GEAR_ROTATION} degrees of gear rotation")
            angle_to_pulse_width(pi, right_angle)
            time.sleep(2)

            # Get servo sensor reading
            print("Taking servo sensor reading...")
            servo_right = get_sensor_voltage(servo_sensor)
            print(f"Right position: Servo={servo_right:.3f}V")

            # Return to center
            angle_to_pulse_width(pi, servo_center)

            # Now calibrate the lever sensor
            print("\nCalibrating lever sensor...")
            print("Let the lever rest in its natural position (0 degrees)...")
            time.sleep(1)  # Give a moment to ensure lever is at rest

            # Get lever sensor reading at rest
            print("Taking lever sensor reading at rest position...")
            lever_rest = get_sensor_voltage(lever_sensor)
            print(f"Lever rest position: {lever_rest:.3f}V")

            # Ask user to extend lever to maximum position
            print("\nPlease move the lever to its maximum extended position and press Enter...")
            input()  # Wait for user to press Enter

            # Get lever sensor reading at maximum extension
            print("Taking lever sensor reading at maximum position...")
            lever_max = get_sensor_voltage(lever_sensor)
            print(f"Lever maximum position: {lever_max:.3f}V")

            # Detect and inform about lever direction
            if lever_max > lever_rest:
                print("\nDetected: Lever voltage INCREASES when extended")
            else:
                print("\nDetected: Lever voltage DECREASES when extended")

            # Set calibration values
            servo_min = min(servo_left, servo_center_voltage, servo_right)
            servo_max = max(servo_left, servo_center_voltage, servo_right)

            # Save calibration data
            save_calibration(servo_center, servo_min, servo_max, lever_rest, lever_max)

        print("\nCalibration complete!")
        print(f"Servo center: {servo_center:.1f} degrees")
        print(f"Servo sensor range: {servo_min:.3f}V to {servo_max:.3f}V")
        print(f"Lever sensor range: {lever_rest:.3f}V (rest) to {lever_max:.3f}V (extended)")

        # Task parameters - using values from configuration section
        delivery_zone = DELIVERY_ZONE
        time_for_reward = TIME_FOR_REWARD
        trial_duration = TRIAL_DURATION
        lever_sensitivity = LEVER_SENSITIVITY

        # Configure disturbance parameters
        print("\nSelect disturbance style:")
        print("1. Sinusoidal oscillation (sin)")
        print("2. No disturbance (none)")
        disturbance_choice = input("Enter choice (1/2): ").strip()

        if disturbance_choice == "2":
            disturbance_style = "none"
            print("Selected: No disturbance")
        else:
            disturbance_style = "sin"
            print("Selected: Sinusoidal oscillation")

            # Allow customization of disturbance parameters
            print("\nDisturbance parameters:")
            print(f"Default magnitude: {DISTURBANCE_MAGNITUDE} degrees")
            print(f"Default period: {DISTURBANCE_PERIOD} seconds")

            custom_params = input("Do you want to customize these parameters? (y/n): ").strip().lower()
            if custom_params == 'y' or custom_params == 'yes':
                try:
                    magnitude_input = input(f"Enter magnitude in degrees (default: {DISTURBANCE_MAGNITUDE}): ").strip()
                    disturbance_magnitude = float(magnitude_input) if magnitude_input else DISTURBANCE_MAGNITUDE

                    period_input = input(f"Enter period in seconds (default: {DISTURBANCE_PERIOD}): ").strip()
                    disturbance_period = float(period_input) if period_input else DISTURBANCE_PERIOD
                except ValueError:
                    print("Invalid input. Using default values.")
                    disturbance_magnitude = DISTURBANCE_MAGNITUDE
                    disturbance_period = DISTURBANCE_PERIOD
            else:
                disturbance_magnitude = DISTURBANCE_MAGNITUDE
                disturbance_period = DISTURBANCE_PERIOD

        print(f"\nTask parameters:")
        print(f"Delivery zone: ±{delivery_zone} degrees around center")
        print(f"Time for reward: {time_for_reward} seconds")
        print(f"Trial duration: {trial_duration} seconds")
        print(f"Disturbance style: {disturbance_style}")
        if disturbance_style == "sin":
            print(f"Disturbance magnitude: {disturbance_magnitude} degrees")
            print(f"Disturbance period: {disturbance_period} seconds")
        print(f"Lever sensitivity: {lever_sensitivity}")

        # This code only runs on Raspberry Pi, so we don't need to check again

        print("\nStarting task. Press Ctrl+C to exit.")

        # Variables to track state
        start_time = time.perf_counter()
        reward_start_time = time.perf_counter()
        in_zone = False
        next_data_time = start_time + 0.1  # First data point at 0.1s
        consecutive_errors = 0  # Counter for consecutive I2C errors

        try:
            while True:
                current_time = time.perf_counter() - start_time

                # Check if trial duration has been reached
                if current_time > trial_duration:
                    print("\nTrial duration reached. Ending task.")
                    break

                # Get current sensor readings with error handling
                try:
                    servo_voltage = servo_sensor.voltage
                    lever_voltage = lever_sensor.voltage
                except OSError as e:
                    # Handle I2C communication errors
                    print(f"\nI2C error: {e}. Retrying in 0.5 seconds...")

                    # Try to reset the servo to a safe position during errors
                    try:
                        # Move to center position during errors to prevent erratic movement
                        angle_to_pulse_width(pi, servo_center)
                    except:
                        pass

                    # Count consecutive errors
                    consecutive_errors += 1

                    # If too many consecutive errors, offer to run diagnostics
                    if consecutive_errors > 10:
                        print("\n\nToo many consecutive I2C errors. Would you like to:")
                        print("1. Run diagnostics")
                        print("2. Continue trying")
                        print("3. Exit program")
                        choice = input("Enter choice (1/2/3): ").strip()

                        if choice == "1":
                            diagnose_i2c()
                            consecutive_errors = 0
                        elif choice == "3":
                            print("Exiting program due to too many I2C errors.")
                            break
                        else:
                            print("Continuing to try...")
                            consecutive_errors = 0

                    time.sleep(0.5)
                    continue  # Skip this iteration and try again

                # Reset consecutive error counter when successful
                consecutive_errors = 0

                # Convert to angles
                # Servo angle should be the actual measured angle of the servo
                # We need to map the voltage to the full range of servo motion
                servo_angle = voltage_to_degrees(servo_voltage, servo_min, servo_max,
                                               servo_center-MAX_SERVO_ROTATION, servo_center+MAX_SERVO_ROTATION)

                # Lever angle is from 0 to 45 degrees
                # Note: We're handling the case where the encoder might rotate in the opposite direction
                # by checking if lever_max is less than lever_rest (voltage decreases as lever extends)
                lever_angle = voltage_to_degrees(lever_voltage, lever_rest, lever_max, 0, 45)

                # Calculate disturbance angle (centered at servo_center)
                if disturbance_style == "sin":
                    disturbance_angle = get_disturbance(disturbance_style, current_time, servo_center,
                                                      disturbance_magnitude, disturbance_period)
                else:
                    disturbance_angle = get_disturbance(disturbance_style, current_time, servo_center)

                # Calculate the correction based on lever position
                # When lever is at rest (0°), servo should go to one extreme (servo_center - MAX_ROTATION)
                # When lever is fully down (45°), servo should go to the other extreme (servo_center + MAX_ROTATION)
                # This creates a full range of motion from one extreme to the other based on lever position
                # The movement is reversed from the original behavior

                # Map lever angle (0-45) to servo offset (-MAX_ROTATION to +MAX_ROTATION)
                # When lever is at 0, offset is -MAX_SERVO_ROTATION (reversed from original)
                # When lever is at 45, offset is +MAX_SERVO_ROTATION (reversed from original)
                # This ensures the servo moves exactly MAX_SERVO_ROTATION degrees in each direction
                # which corresponds to MAX_GEAR_ROTATION degrees at the gear output
                lever_offset = -MAX_SERVO_ROTATION + (2 * MAX_SERVO_ROTATION * lever_angle / 45)

                # Calculate final angle based on disturbance and lever position
                if disturbance_style == "none":
                    # When no disturbance, final angle is just based on lever position
                    final_angle = servo_center + lever_offset
                else:
                    # When using disturbance, add it to the lever offset
                    final_angle = disturbance_angle + lever_offset

                # Ensure we don't exceed the safe range (servo_center ± MAX_SERVO_ROTATION)
                final_angle = max(servo_center - MAX_SERVO_ROTATION, min(servo_center + MAX_SERVO_ROTATION, final_angle))

                # Ensure final angle is within valid range for the servo (0-180 degrees)
                # This is a secondary safety check after the MAX_SERVO_ROTATION limit
                final_angle = max(0, min(180, final_angle))

                # Set servo position
                angle_to_pulse_width(pi, final_angle)

                # Check if it's time to record data
                if time.perf_counter() - next_data_time >= 0:
                    # Check if in delivery zone (centered around servo_center)
                    reward_delivered = 0
                    if final_angle >= servo_center - delivery_zone and final_angle <= servo_center + delivery_zone:
                        if not in_zone:
                            reward_start_time = time.perf_counter()
                            in_zone = True
                        elif time.perf_counter() - reward_start_time >= time_for_reward:
                            print("\nReward delivered!")
                            dispense_reward(pi)
                            reward_start_time = time.perf_counter()
                            reward_delivered = 1
                    else:
                        in_zone = False

                    # Write data to file
                    data_writer.writerow([
                        f"{current_time:.1f}",
                        f"{servo_angle:.1f}",
                        f"{lever_angle:.1f}",
                        f"{disturbance_angle:.1f}",
                        f"{final_angle:.1f}",
                        reward_delivered
                    ])
                    data_file.flush()  # Ensure data is written to disk

                    # No real-time plotting in this version

                    # Set next data collection time
                    next_data_time = time.perf_counter() + 0.1

                # Update DAC output with the disturbance angle
                update_dac_output(disturbance_angle)

                # Display current status
                print(f"\rTime: {current_time:.1f}s | Servo: {servo_angle:.1f}° | Lever: {lever_angle:.1f}° | Disturbance: {disturbance_angle:.1f}° | Final: {final_angle:.1f}°", end="", flush=True)

                # Small delay to prevent CPU overuse
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nTask interrupted by user.")
        finally:
            # Center the servo at the calibrated center position and clean up
            angle_to_pulse_width(pi, servo_center)
            time.sleep(0.5)

            # Reset DAC output to 0V
            if dac_available:
                try:
                    dac.raw_value = 0
                    print("DAC output reset to 0V")
                except:
                    pass

            # Close the plot if it exists
            if plot_fig is not None:
                try:
                    plt.close(plot_fig)
                    print("Plot closed")
                except:
                    pass

            pi.stop()
            print(f"\nData saved to {data_filename}")

            # Get the Raspberry Pi's IP address
            try:
                # Try to get the IP address using the hostname command
                ip_address = os.popen("hostname -I | awk '{print $1}'").read().strip()

                # If we got an IP address, print the SCP command
                if ip_address:
                    # Create the SCP command with the actual IP and filename
                    scp_command = f"scp pi@{ip_address}:{data_filename} ./"
                    print(f"\nTo retrieve the data file, run this command on your computer:")
                    print(f"\n  {scp_command}")
                    print("\nThis will copy the file to your current directory.")
                    print("You may be prompted for the Raspberry Pi password.")
            except:
                # If we couldn't get the IP address, provide a generic command
                print("\nTo retrieve the data file, run this command on your computer:")
                print(f"\n  scp pi@[RaspberryPi_IP]:{data_filename} ./")
                print("\nReplace [RaspberryPi_IP] with your Raspberry Pi's IP address.")

if __name__ == "__main__":
    main()
