import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import pigpio
import sys

# Constants for Servo
SERVO_PIN = 17  # GPIO pin for servo control
MIN_PULSE_WIDTH = 500  # Pulse width in microseconds for minimum position
MAX_PULSE_WIDTH = 2500  # Pulse width in microseconds for maximum position

def angle_to_pulse_width(pi, angle):
    """Convert angle to servo pulse width"""
    angle = max(0, min(180, angle))
    pulse_width = MIN_PULSE_WIDTH + (angle / 180) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)
    return pulse_width

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon")
    print("Run 'sudo pigpiod' and try again")
    sys.exit(1)

# Initialize I2C bus and ADS1115
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
ads.gain = 1  # Set gain for 5V sensor

# Set up all four channels
channels = [
    AnalogIn(ads, ADS.P0),  # Channel 0 (lever sensor)
    AnalogIn(ads, ADS.P1),  # Channel 1 (servo sensor)
    AnalogIn(ads, ADS.P2),  # Channel 2
    AnalogIn(ads, ADS.P3)   # Channel 3
]

print("Sensor Monitor with Servo Control - Press Ctrl+C to exit")
print("-------------------------------------------------------")
print("Enter an angle (0-180) to move the servo, or just press Enter to update readings")
print("Type 'q' to quit")

try:
    # Start with servo at center position
    current_angle = 90
    angle_to_pulse_width(pi, current_angle)
    print(f"Servo set to {current_angle} degrees")

    while True:
        # Print header
        print("\nChannel | Voltage  | Raw Value")
        print("-------------------------------")

        # Read and display all channels
        for i, channel in enumerate(channels):
            print(f"  {i}     | {channel.voltage:.4f}V | {channel.value}")

        # Get user input for servo control
        user_input = input("\nEnter angle (0-180) or press Enter to update readings: ").strip()

        if user_input.lower() == 'q':
            break

        if user_input:
            try:
                angle = float(user_input)
                if 0 <= angle <= 180:
                    current_angle = angle
                    pulse_width = angle_to_pulse_width(pi, current_angle)
                    print(f"Servo moved to {current_angle} degrees (pulse width: {pulse_width}µs)")
                else:
                    print("Angle must be between 0 and 180 degrees")
            except ValueError:
                print("Invalid input. Enter a number between 0 and 180.")

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    # Clean up
    pi.set_servo_pulsewidth(SERVO_PIN, 0)  # Turn off servo
    pi.stop()
    print("Servo turned off")
