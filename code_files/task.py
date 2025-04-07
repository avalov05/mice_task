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
ADS1115_CHANNEL = 0  # Channel on the ADS1115 where hall sensor is connected

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
    
def hall_sensor_to_angle(hall_sensor, calibration_data):
    """
    Converts a hall sensor reading to an angle
    """
    
    if hall_sensor.value > calibration_data[0] and hall_sensor.value < calibration_data[1]:
        angle = 30 +  (hall_sensor.value - calibration_data[0]) / (abs(calibration_data[1] - calibration_data[0])/MAX_ROTATION)
    elif hall_sensor.value > calibration_data[1] and hall_sensor.value < calibration_data[2]:
        angle = 90 + (hall_sensor.value - calibration_data[1]) / (abs(calibration_data[2] - calibration_data[1])/MAX_ROTATION)
    else:
        angle = 90

    return angle

def green_sensor_to_angle(green_sensor, calibration_data):
    """
    Converts a green sensor reading to an angle
    """
    if green_sensor.value > calibration_data[3] and green_sensor.value < calibration_data[4]:
        angle = 30 +  (green_sensor.value - calibration_data[3]) / (abs(calibration_data[4] - calibration_data[3])/MAX_ROTATION)
    elif green_sensor.value > calibration_data[4] and green_sensor.value < calibration_data[5]:
        angle = 90 + (green_sensor.value - calibration_data[4]) / (abs(calibration_data[5] - calibration_data[4])/MAX_ROTATION)
    else:
        angle = 90

    return angle

def calibration(pi, hall_sensor, green_sensor):

    #postions to calibrate
    POSTIONS = ["LEFT", "MIDDLE", "RIGHT"]
    calibration_data = []

    for position in POSTIONS:
        input(f"Please move the lever to the {position} position and press Enter...")

        #take 20 readings to get values despite noise
        readings = []
        for i in range(20):
            readings.append(hall_sensor.value)
            time.sleep(0.05)
            print(".", end="", flush=True)

        # Remove outliers and average
        readings.sort()
        trimmed_readings = readings[5:-5]  # Remove 5 lowest and 5 highest values
        avg_reading = sum(trimmed_readings) / len(trimmed_readings)

        #append to calibration data
        calibration_data.append(avg_reading)
        print(f"\nRecorded {position} position: {avg_reading:.1f}")

    #center spout and take readings rotate servo to its center
    print("Reading servo sensor at leftmost position...")
    left_angle = 90 - MAX_ROTATION
    angle_to_pulse_width(pi, left_angle)
    time.sleep(2)
    calibration_data.append(green_sensor.value)
    print(f"\nRecorded position: {calibration_data[-1]}")
    
    print("Reading servo sensor at center position...")
    center_angle = 90
    angle_to_pulse_width(pi, center_angle)
    time.sleep(2)
    calibration_data.append(green_sensor.value)
    print(f"\nRecorded position: {calibration_data[-1]}")
    
    print("Reading servo sensor at rightmost position...")
    right_angle = 90 + MAX_ROTATION
    angle_to_pulse_width(pi, right_angle)
    time.sleep(2)
    calibration_data.append(green_sensor.value)
    print(f"\nRecorded position: {calibration_data[-1]}")

    
    #make sure the sensor is not attached where the values can go from 0 to 25000, aka when its rotating from 0 to 25000 and drops to 0
    if calibration_data[0] < calibration_data[1] < calibration_data[2]:
        print("Calibration successful!")
    elif calibration_data[0] > calibration_data[1] > calibration_data[2]:
        print("Calibration successful!")
    else:
        print("Calibration failed. Please rotate the sensor without rotating the lever and try again.")
        exit(2)

    return calibration_data

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
    hall_sensor = AnalogIn(ads, ADS.P0 + ADS1115_CHANNEL)
    green_sensor = AnalogIn(ads, ADS.P1 + ADS1115_CHANNEL)
    # Calibrate the sensor
    calibration_data = calibration(pi, hall_sensor, green_sensor)

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

        #get hall sensor angle
        hall_sensor_angle = hall_sensor_to_angle(hall_sensor, calibration_data)

        #get angle to move to
        final_angle = disturbance_angle + (hall_sensor_angle-90)

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

        #get green sensor angle
        green_sensor_angle = green_sensor_to_angle(green_sensor, calibration_data)

        #save data
        data[current_time] = {
            "disturbance_angle": disturbance_angle,
            "hall_sensor_angle": hall_sensor_angle,
            "final_angle": green_sensor_angle,
            "in_zone": in_zone
        }

        #send data to laptop
        
        # Add a small delay to prevent CPU overload
        time.sleep(0.01)

    send_data(data, "172.20.10.10", 12345)

if __name__ == "__main__":
    main()


