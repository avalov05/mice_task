import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import pigpio
import sys

# Constants for Servo
SERVO_PIN = 17
MIN_PULSE_WIDTH = 500
MAX_PULSE_WIDTH = 2500

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon")
    sys.exit(1)

# Initialize I2C bus and ADS1115
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
ads.gain = 1

# Set up sensors
lever_sensor = AnalogIn(ads, ADS.P0)  # Hall sensor for lever
servo_sensor = AnalogIn(ads, ADS.P1)  # Green sensor for servo

print("Super Quick Sensor Test - Press Ctrl+C to exit")
print("Enter angle (0-180) to move servo, or 'q' to quit")

try:
    # Start with servo at center
    pi.set_servo_pulsewidth(SERVO_PIN, 1500)  # Center position

    while True:
        # Read and display sensor values
        print(f"Lever: {lever_sensor.voltage:.4f}V | Servo: {servo_sensor.voltage:.4f}V")

        # Get user input for servo control
        user_input = input("Enter angle (0-180): ").strip()

        if user_input.lower() == 'q':
            break

        try:
            angle = float(user_input)
            if 0 <= angle <= 180:
                # Convert angle to pulse width
                pulse_width = MIN_PULSE_WIDTH + (angle / 180) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
                pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)
                print(f"Moved servo to {angle} degrees")
            else:
                print("Angle must be between 0 and 180")
        except ValueError:
            print("Invalid input. Enter a number between 0 and 180.")

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    # Clean up
    pi.set_servo_pulsewidth(SERVO_PIN, 0)
    pi.stop()
