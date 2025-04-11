import pigpio #type: ignore
import time
import board #type: ignore
import busio #type: ignore  
import adafruit_ads1x15.ads1115 as ADS #type: ignore
from adafruit_ads1x15.analog_in import AnalogIn #type: ignore
import numpy as np #type: ignore
import socket
import json

'''
    Constants
'''

# Constants for Servo
MAX_ROTATION = 60 #degrees
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

def get_disturbance(disturbance_style, current_time):
    """
    Returns the angle that the disturbance should be at given the current time
    """
    if disturbance_style == "sin":
        return np.sin(current_time) * MAX_ROTATION + 90
    
def calibration(pi, green_sensor, hall_sensor):
    """
    Calibrates the green sensor using resting position and 90-degree clockwise rotation
    """
    # First calibrate resting position
    input("Please leave the lever at its resting position (leftmost) and press Enter...")
    
    # Take multiple readings at rest position to get a stable value
    readings = []
    for i in range(20):
        readings.append(green_sensor.value)
        time.sleep(0.05)
        print(".", end="", flush=True)

    # Remove outliers and average
    readings.sort()
    trimmed_readings = readings[5:-5]  # Remove 5 lowest and 5 highest values
    rest_position = sum(trimmed_readings) / len(trimmed_readings)
    print(f"\nRecorded rest position: {rest_position:.1f}")

    # Now calibrate 90-degree position
    input("\nPlease rotate the lever 90 degrees clockwise and press Enter...")
    
    readings = []
    for i in range(20):
        readings.append(green_sensor.value)
        time.sleep(0.05)
        print(".", end="", flush=True)

    readings.sort()
    trimmed_readings = readings[5:-5]
    rotated_position = sum(trimmed_readings) / len(trimmed_readings)
    print(f"\nRecorded 90-degree position: {rotated_position:.1f}")

    # Return calibration points
    return [rest_position, rotated_position]

def green_sensor_to_angle(green_sensor, calibration_data):
    """
    Converts green sensor reading to angle where:
    - Rest position (leftmost) = 30 degrees
    - 90 degrees clockwise = 120 degrees
    """
    rest_value, rotated_value = calibration_data
    
    # Calculate the voltage range for 90 degrees of motion
    voltage_range = abs(rotated_value - rest_value)
    
    # Calculate current position relative to rest position
    relative_position = green_sensor.value - rest_value
    
    # Convert to angle (30 degrees at rest, 120 degrees at full rotation)
    angle = 30 + (relative_position / voltage_range) * 90
    
    # Clamp the angle between 30 and 120 degrees
    return max(30, min(120, angle))

def hall_sensor_to_angle(hall_sensor, calibration_data):
    """
    Converts hall sensor reading to angle using the same principle as green sensor
    """
    rest_value, rotated_value = calibration_data
    
    voltage_range = abs(rotated_value - rest_value)
    relative_position = hall_sensor.value - rest_value
    
    angle = 30 + (relative_position / voltage_range) * 90
    
    return max(30, min(120, angle))

def send_data(data, ip, port):
    """
    Sends all collected data to the server
    """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((ip, port))
        
        # Convert data to JSON string
        json_data = json.dumps(data)
        
        # Send the complete dataset with proper encoding
        s.sendall(json_data.encode('utf-8'))
        
        # Close the connection
        s.close()
        print(f"Successfully sent {len(data)} data points")
    except Exception as e:
        print(f"Error sending data: {e}")

def main():
    # Initialize pigpio
    pi = pigpio.pi()
    if not pi.connected:
        print("Failed to connect to pigpio daemon")
        exit(1)

    # Initialize I2C bus and ADS1115
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    # Set the gain to capture smaller voltage changes
    ads.gain = 2  # Options: 2/3, 1, 2, 4, 8, 16 (higher = more precision for smaller voltages)
    green_sensor = AnalogIn(ads, ADS.P0 + ADS1115_CHANNEL)
    hall_sensor = AnalogIn(ads, ADS.P1 + ADS1115_CHANNEL)
    # Calibrate the sensor
    calibration_data = calibration(pi, green_sensor, hall_sensor)

    """
    Task Parameters
    """
    delivery_zone = 10 #degrees
    time_for_reward = 1 #seconds
    trial_duration = 20 #seconds
    disturbance_style = "sin" #'step','ramp','sin','tri','random','stepDelay'

    """
    Task
    """
    start_time = time.perf_counter()
    reward_start_time = time.perf_counter()
    in_zone = False  # Track whether we're in the delivery zone

    data = {}
    
    while True:
        #get current time
        current_time = time.perf_counter() - start_time
        #check if trial is over
        if current_time > trial_duration:
            break

        #get disturbance angle
        disturbance_angle = get_disturbance(disturbance_style, current_time)

        #get green sensor angle
        green_sensor_angle = green_sensor_to_angle(green_sensor, calibration_data)

        #get angle to move to
        final_angle = disturbance_angle + (green_sensor_angle-90)

        angle_to_pulse_width(pi, final_angle)

        #reward check
        if final_angle >= 90 - delivery_zone and final_angle <= 90 + delivery_zone:
            # We're in the delivery zone
            if not in_zone:
                # Just entered the zone, start timing
                reward_start_time = time.perf_counter()
                in_zone = True
            elif time.perf_counter() - reward_start_time >= time_for_reward:
                # We've been in the zone long enough
                print("Reward delivered")
                # Reset timer after delivering reward
                reward_start_time = time.perf_counter()
        else:
            # We're outside the delivery zone
            in_zone = False

        #get hall sensor angle
        hall_sensor_angle = hall_sensor_to_angle(hall_sensor, calibration_data)

        #save data
        data[current_time] = {
            "disturbance_angle": disturbance_angle,
            "green_sensor_angle": green_sensor_angle,
            "final_angle": hall_sensor_angle,
            "in_zone": in_zone
        }

        # Add a small delay to prevent CPU overload
        time.sleep(0.01)

    send_data(data, "172.20.10.10", 12345)

if __name__ == "__main__":
    main() 