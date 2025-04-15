import pigpio #type: ignore
import time
import board #type: ignore
import busio #type: ignore  
import adafruit_ads1x15.ads1115 as ADS #type: ignore
from adafruit_ads1x15.analog_in import AnalogIn #type: ignore
import numpy as np #type: ignore
import socket
import json
import signal
import sys

'''
    Constants
'''

# Constants for Servo
MAX_ROTATION = 30 #degrees (changed to 30 degrees for this task)
SERVO_PIN = 17  # GPIO pin for servo control
MIN_PULSE_WIDTH = 500  # Pulse width in microseconds for minimum position
MAX_PULSE_WIDTH = 2500  # Pulse width in microseconds for maximum position

# Constants for ADS1115
ADS1115_CHANNEL = 0  # Channel on the ADS1115 

def angle_to_pulse_width(pi, angle):
    """
    Convert an angle in degrees to a servo pulse width in microseconds.
    
    Args:
        angle: Angle in degrees (0-180, where 0 is leftmost, 90 is center, 180 is rightmost)
    """
    # Ensure angle is within valid range
    angle = max(0, min(180, angle))
    
    # Convert angle to pulse width
    pulse_width = MIN_PULSE_WIDTH + (angle / 180) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)

def get_sensor_reading(green_sensor, num_samples=20):
    """
    Get averaged sensor reading with outlier removal
    """
    readings = []
    for _ in range(num_samples):
        readings.append(green_sensor.value)
        time.sleep(0.05)
        print(".", end="", flush=True)

    readings.sort()
    trimmed_readings = readings[5:-5]  # Remove 5 lowest and 5 highest values
    return sum(trimmed_readings) / len(trimmed_readings)

def calibration(pi, green_sensor, hall_sensor):
    """
    Calibrates using servo positions:
    - 30 degrees left of center (60 degrees)
    - Center (90 degrees)
    - 30 degrees right of center (120 degrees)
    """
    print("\nStarting automatic calibration sequence...")
    calibration_points = {}
    
    # Center position (90 degrees)
    print("\nMoving to left position...")
    angle_to_pulse_width(pi, 60)
    time.sleep(1)  # Wait for servo to settle
    left_value = get_sensor_reading(green_sensor)
    calibration_points['left'] = left_value
    print(f"\nLeft position value: {left_value:.1f}")

    # Left position (60 degrees)
    print("\nMoving to center position...")
    angle_to_pulse_width(pi, 90)
    time.sleep(1)
    center_value = get_sensor_reading(green_sensor)
    calibration_points['center'] = center_value
    print(f"\nCenter position value: {center_value:.1f}")

    # Right position (120 degrees)
    print("\nMoving to right position...")
    angle_to_pulse_width(pi, 120)
    time.sleep(1)
    right_value = get_sensor_reading(green_sensor)
    calibration_points['right'] = right_value
    print(f"\nRight position value: {right_value:.1f}")

    # Return to center
    angle_to_pulse_width(pi, 90)
    
    return calibration_points

def green_sensor_to_angle(green_sensor, calibration_points):
    """
    Converts green sensor reading to angle using three-point calibration
    """
    current_value = green_sensor.value
    
    # If value is between left and center
    if current_value <= calibration_points['center']:
        # Map between 60 and 90 degrees
        ratio = (current_value - calibration_points['left']) / (calibration_points['center'] - calibration_points['left'])
        angle = 60 + (ratio * 30)
    else:
        # Map between 90 and 120 degrees
        ratio = (current_value - calibration_points['center']) / (calibration_points['right'] - calibration_points['center'])
        angle = 90 + (ratio * 30)
    
    return max(60, min(120, angle))

def hall_sensor_to_angle(hall_sensor, calibration_points):
    """
    Converts hall sensor reading to angle.
    Resting position = Maximum right
    Resting position + 20° = Center (90°)
    Resting position + 40° = Maximum left
    """
    current_value = hall_sensor.value
    resting_value = calibration_points['right']  # This is the resting position (maximum right)
    
    # Calculate how far we are from resting position
    delta = current_value - resting_value
    
    # Map the delta to degrees
    # At resting (delta = 0) we want maximum right (110°)
    # At resting + 20° we want center (90°)
    # At resting + 40° we want maximum left (70°)
    # Linear mapping between these points
    angle = 110 - (delta / (calibration_points['left'] - resting_value)) * 40
    
    return max(70, min(110, angle))

def get_disturbance(disturbance_style, current_time):
    """
    Returns the angle that the disturbance should be at given the current time
    """
    if disturbance_style == "sin":
        return np.sin(current_time) * MAX_ROTATION + 90

def send_data(data, ip, port):
    """
    Sends all collected data to the server
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((ip, port))
        json_data = json.dumps(data)
        s.sendall(json_data.encode('utf-8'))
        s.close()
        print(f"Successfully sent {len(data)} data points")
    except Exception as e:
        print(f"Error sending data: {e}")

def signal_handler(sig, frame):
    """
    Handle Ctrl+C by centering the servo before exiting
    """
    print("\nCentering servo and exiting...")
    pi = pigpio.pi()
    if pi.connected:
        angle_to_pulse_width(pi, 90)  # Center position
        time.sleep(0.5)  # Give servo time to move
        pi.stop()
    sys.exit(0)

def main():
    # Set up signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize pigpio
    pi = pigpio.pi()
    if not pi.connected:
        print("Failed to connect to pigpio daemon")
        exit(1)

    # Initialize I2C bus and ADS1115
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    ads.gain = 1  # Changed from 2 to 1 for 5V sensor
    green_sensor = AnalogIn(ads, ADS.P1 + ADS1115_CHANNEL)
    hall_sensor = AnalogIn(ads, ADS.P0 + ADS1115_CHANNEL)
    
    # Calibrate the sensors
    calibration_points = calibration(pi, green_sensor, hall_sensor)

    """
    Task Parameters
    """
    delivery_zone = 10 #degrees
    time_for_reward = 1 #seconds
    trial_duration = 20 #seconds
    disturbance_style = "sin"
    lever_sensitivity = 2

    """
    Task
    """
    start_time = time.perf_counter()
    reward_start_time = time.perf_counter()
    in_zone = False

    data = {}
    
    while True:
        current_time = time.perf_counter() - start_time
        if current_time > trial_duration:
            break

        disturbance_angle = get_disturbance(disturbance_style, current_time)
        green_sensor_angle = green_sensor_to_angle(green_sensor, calibration_points)
        hall_sensor_angle = hall_sensor_to_angle(hall_sensor, calibration_points)
        
        # Calculate the correction needed based on hall sensor
        # If hall sensor is at 90° (center), no correction needed
        # If hall sensor is > 90°, we need to move servo left to compensate
        # If hall sensor is < 90°, we need to move servo right to compensate
        correction = lever_sensitivity * (hall_sensor_angle - 90)
        
        # Apply the correction to the final angle
        final_angle = disturbance_angle - correction
        angle_to_pulse_width(pi, final_angle)

        if final_angle >= 90 - delivery_zone and final_angle <= 90 + delivery_zone:
            if not in_zone:
                reward_start_time = time.perf_counter()
                in_zone = True
            elif time.perf_counter() - reward_start_time >= time_for_reward:
                print("Reward delivered")
                reward_start_time = time.perf_counter()
        else:
            in_zone = False

        data[current_time] = {
            "disturbance_angle": disturbance_angle,
            "green_sensor_angle": green_sensor_angle,
            "hall_sensor_angle": hall_sensor_angle,
            "final_angle": final_angle,
            "in_zone": in_zone
        }

        time.sleep(0.01)

    send_data(data, "172.20.10.10", 12345)

if __name__ == "__main__":
    main()