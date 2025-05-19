import pigpio
import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import signal
import sys
import select
import os
import datetime
import csv

'''
    Constants
'''
# GPIO pin for the green LED (reward)
GREEN_LED_PIN = 27

# Constants for ADS1115
ADS1115_CHANNEL = 0  # Channel on the ADS1115

# Buffer zone for analog input variability (in volts)
BUFFER_ZONE = 0.1  # Adjust this value based on your sensor's variability

# Constants for lever angle conversion
MIN_ANGLE = 0    # Minimum angle in degrees
MAX_ANGLE = 45  # Maximum angle in degrees
# These values will be set during calibration
MIN_VOLTAGE = 0.0  # Minimum voltage (will be updated during calibration)
MAX_VOLTAGE = 3.3  # Maximum voltage (will be updated during calibration)

def get_sensor_voltage(sensor, num_samples=20):
    """
    Get averaged sensor voltage with outlier removal
    """
    readings = []
    for _ in range(num_samples):
        readings.append(sensor.voltage)  # Use voltage instead of raw value
        time.sleep(0.05)
        print(".", end="", flush=True)

    readings.sort()
    trimmed_readings = readings[5:-5]  # Remove 5 lowest and 5 highest values
    return sum(trimmed_readings) / len(trimmed_readings)

def flash_led(pi, duration=0.007):
    """
    Flash the green LED for the specified duration
    """
    pi.write(GREEN_LED_PIN, 1)  # Turn on LED
    time.sleep(duration)
    pi.write(GREEN_LED_PIN, 0)  # Turn off LED

def voltage_to_degrees(voltage, resting_voltage, max_voltage):
    """
    Convert a voltage to degrees, where resting_voltage = 0 degrees
    and max_voltage = MAX_ANGLE degrees. Works regardless of whether
    max_voltage is greater than or less than resting_voltage.

    Args:
        voltage: Voltage from the sensor
        resting_voltage: Voltage at resting position (0 degrees)
        max_voltage: Voltage at maximum position (MAX_ANGLE degrees)

    Returns:
        Angle in degrees
    """
    # Determine if voltage increases or decreases as lever rotates
    if max_voltage > resting_voltage:  # Voltage increases with rotation
        # If voltage is at or below resting, return 0 degrees
        if voltage <= resting_voltage:
            return 0.0

        # If voltage is at or above max, return MAX_ANGLE
        if voltage >= max_voltage:
            return float(MAX_ANGLE)

        # Calculate degrees based on how far the voltage is from resting toward max_voltage
        return ((voltage - resting_voltage) / (max_voltage - resting_voltage)) * MAX_ANGLE

    else:  # Voltage decreases with rotation (max_voltage < resting_voltage)
        # If voltage is at or above resting, return 0 degrees
        if voltage >= resting_voltage:
            return 0.0

        # If voltage is at or below max, return MAX_ANGLE
        if voltage <= max_voltage:
            return float(MAX_ANGLE)

        # Calculate degrees based on how far the voltage is from resting toward max_voltage
        return ((resting_voltage - voltage) / (resting_voltage - max_voltage)) * MAX_ANGLE

# Global variable to track if data file is open
data_file = None

def signal_handler(sig, frame):
    """
    Handle Ctrl+C by cleaning up GPIO before exiting
    """
    print("\nCleaning up GPIO and exiting...")
    pi = pigpio.pi()
    if pi.connected:
        pi.write(GREEN_LED_PIN, 0)  # Ensure LED is off
        pi.stop()
    sys.exit(0)

def save_calibration(min_voltage, max_voltage, resting_voltage):
    """
    Save calibration data to a file
    """
    # Create calibration directory if it doesn't exist
    calib_dir = os.path.join(os.getcwd(), "calibration_data")
    if not os.path.exists(calib_dir):
        os.makedirs(calib_dir)

    # Create calibration file
    calib_file = os.path.join(calib_dir, "lever_calibration.csv")
    with open(calib_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["min_voltage", "max_voltage", "resting_voltage"])
        writer.writerow([f"{min_voltage:.6f}", f"{max_voltage:.6f}", f"{resting_voltage:.6f}"])

    print(f"Calibration data saved to {calib_file}")

def load_calibration():
    """
    Load calibration data from a file

    Returns:
        tuple: (min_voltage, max_voltage, resting_voltage) or None if file doesn't exist
    """
    calib_file = os.path.join(os.getcwd(), "calibration_data", "lever_calibration.csv")
    if not os.path.exists(calib_file):
        return None

    try:
        with open(calib_file, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # Skip header
            row = next(reader)
            return float(row[0]), float(row[1]), float(row[2])
    except (IndexError, ValueError, StopIteration):
        print("Error reading calibration file. Will perform new calibration.")
        return None

def get_trial_number():
    """
    Get the next trial number by checking existing data files
    """
    data_dir = os.path.join(os.getcwd(), "simple lever press data")
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)
        return 1

    # List all files in the directory
    files = os.listdir(data_dir)
    trial_numbers = []

    # Extract trial numbers from filenames
    for file in files:
        if file.startswith("trial_") and file.endswith(".txt"):
            try:
                trial_num = int(file.split("_")[1].split(".")[0])
                trial_numbers.append(trial_num)
            except (ValueError, IndexError):
                continue

    # Return the next trial number
    return max(trial_numbers, default=0) + 1

def main():
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

    # Initialize I2C bus and ADS1115
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    ads.gain = 1  # Set gain for 5V sensor
    lever_sensor = AnalogIn(ads, ADS.P0 + ADS1115_CHANNEL)  # Assuming lever is connected to P0

    # Create data directory if it doesn't exist
    data_dir = os.path.join(os.getcwd(), "simple lever press data")
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)

    # Get trial number and create data file
    trial_number = get_trial_number()
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    data_filename = os.path.join(data_dir, f"trial_{trial_number}_{timestamp}.txt")

    print(f"Data will be saved to: {data_filename}")

    # Create and open data file
    with open(data_filename, 'w', newline='') as data_file:
        # Write header
        data_writer = csv.writer(data_file, delimiter='\t')
        data_writer.writerow([
            "Time (s)",
            "Lever Position (degrees)",
            "Reward Delivered (0/1)",
            "Manual Reward (0/1)"
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
            _, max_voltage, resting_voltage = calibration_data  # Ignore min_voltage
            print(f"Using previous calibration:")
            print(f"Resting position (0°): {resting_voltage:.3f}V")
            print(f"Maximum position ({MAX_ANGLE}°): {max_voltage:.3f}V")
        else:
            print("Calibrating lever position...")
            print("Let the lever rest in its natural position (0 degrees)...")
            time.sleep(1)  # Give a moment to ensure lever is at rest
            resting_voltage = get_sensor_voltage(lever_sensor)
            print(f"\nResting position voltage: {resting_voltage:.3f}V")

            print("\nNow, move the lever to its maximum position and press Enter...")
            # Wait for Enter key
            input()
            max_voltage = get_sensor_voltage(lever_sensor)
            print(f"Maximum position voltage: {max_voltage:.3f}V")

            # We'll use resting_voltage as the reference for 0 degrees
            # No need to set min_voltage anymore

            # Save the calibration data
            save_calibration(resting_voltage, max_voltage, resting_voltage)

        # Define the threshold for rotation detection with buffer zone
        # We need to handle both directions of lever movement
        if max_voltage > resting_voltage:
            # Voltage increases with rotation
            min_threshold = resting_voltage  # No need for lower threshold
            max_threshold = resting_voltage + BUFFER_ZONE
        else:
            # Voltage decreases with rotation
            min_threshold = resting_voltage - BUFFER_ZONE
            max_threshold = resting_voltage  # No need for upper threshold

        print("\nCalibration complete!")
        print(f"Resting position (0°): {resting_voltage:.3f}V")
        print(f"Maximum position ({MAX_ANGLE}°): {max_voltage:.3f}V")

        # Test the angle calculation
        test_angle = voltage_to_degrees(resting_voltage, resting_voltage, max_voltage)
        print(f"Test: Resting voltage gives {test_angle:.1f}°")
        test_angle = voltage_to_degrees(max_voltage, resting_voltage, max_voltage)
        print(f"Test: Maximum voltage gives {test_angle:.1f}°")

        # Calculate mid-point voltage
        mid_voltage = (resting_voltage + max_voltage) / 2
        test_angle = voltage_to_degrees(mid_voltage, resting_voltage, max_voltage)
        print(f"Test: Middle voltage ({mid_voltage:.3f}V) gives {test_angle:.1f}°")

        # Show which direction the lever moves
        if max_voltage > resting_voltage:
            print("Lever direction: Voltage INCREASES as lever rotates from rest")
        else:
            print("Lever direction: Voltage DECREASES as lever rotates from rest")

        print("\nReady! Rotate the lever to receive a reward.")
        print("Press ENTER to manually trigger a reward.")

        # Display buffer zone based on lever direction
        if max_voltage > resting_voltage:
            print(f"Buffer zone: Above {max_threshold:.3f}V")
        else:
            print(f"Buffer zone: Below {min_threshold:.3f}V")

        # Variables to track state
        last_reward_time = 0
        reward_cooldown = 0.5  # Seconds between rewards to prevent rapid flashing
        start_time = time.time()
        next_data_time = start_time + 0.1  # First data point at 0.1s

        try:
            while True:
                current_time = time.time()
                elapsed_time = current_time - start_time

                # Read the current lever position (voltage)
                current_voltage = lever_sensor.voltage
                position_degrees = voltage_to_degrees(current_voltage, resting_voltage, max_voltage)
                print(f"\rCurrent position: {position_degrees:.1f}° ({current_voltage:.3f}V)", end="", flush=True)

                # Check if it's time to record data (every 0.1 seconds)
                if current_time >= next_data_time:
                    # Determine if threshold is exceeded based on lever direction
                    if max_voltage > resting_voltage:
                        # Voltage increases with rotation
                        threshold_exceeded = current_voltage > max_threshold
                    else:
                        # Voltage decreases with rotation
                        threshold_exceeded = current_voltage < min_threshold

                    # Initialize reward flags
                    auto_reward = 0
                    manual_reward = 0

                    # Check if lever has been rotated beyond the buffer zone
                    if threshold_exceeded:
                        # Only give reward if cooldown period has passed
                        if current_time - last_reward_time >= reward_cooldown:
                            print(f"\nLever rotated! Current position: {position_degrees:.1f}° ({current_voltage:.3f}V)")
                            flash_led(pi)
                            last_reward_time = current_time
                            auto_reward = 1

                    # Write data to file
                    data_writer.writerow([
                        f"{elapsed_time:.1f}",
                        f"{position_degrees:.1f}",
                        auto_reward,  # Use integer directly, not string
                        manual_reward  # Use integer directly, not string
                    ])
                    data_file.flush()  # Ensure data is written to disk

                    # Set next data collection time
                    next_data_time = current_time + 0.1

                # Check for manual reward trigger (non-blocking)
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    line = sys.stdin.readline()
                    if line.strip() == "":  # Empty line means Enter was pressed
                        if current_time - last_reward_time >= reward_cooldown:
                            print("\nManual reward triggered!")
                            flash_led(pi)
                            last_reward_time = current_time

                            # Get current voltage and convert to degrees
                            current_voltage = lever_sensor.voltage
                            position_degrees = voltage_to_degrees(current_voltage, resting_voltage, max_voltage)

                            # Write manual reward data
                            data_writer.writerow([
                                f"{elapsed_time:.1f}",
                                f"{position_degrees:.1f}",
                                0,  # No auto reward
                                1   # Manual reward
                            ])
                            data_file.flush()

                time.sleep(0.01)  # Small delay to prevent CPU overuse

        except KeyboardInterrupt:
            # This will be caught by the signal handler
            print(f"\nData saved to {data_filename}")
            pass
        finally:
            # Ensure LED is off and clean up
            pi.write(GREEN_LED_PIN, 0)
            pi.stop()

if __name__ == "__main__":
    main()
