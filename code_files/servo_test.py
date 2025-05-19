import pigpio
import time
import sys
import signal

# Constants for Servo
SERVO_PIN = 17  # GPIO pin for servo control
MIN_PULSE_WIDTH = 500  # Pulse width in microseconds for minimum position
MAX_PULSE_WIDTH = 2500  # Pulse width in microseconds for maximum position

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
    return pulse_width

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
        print("Make sure the pigpio daemon is running with: sudo pigpiod")
        exit(1)
    
    print("Servo Test Program")
    print("-----------------")
    print("This program will test the servo motor connected to GPIO pin", SERVO_PIN)
    print("Press Ctrl+C to exit at any time")
    print()
    
    try:
        # Test 1: Move to center position
        print("Test 1: Moving to center position (90 degrees)")
        pulse_width = angle_to_pulse_width(pi, 90)
        print(f"Pulse width: {pulse_width} µs")
        time.sleep(2)
        
        # Test 2: Move to left position
        print("Test 2: Moving to left position (0 degrees)")
        pulse_width = angle_to_pulse_width(pi, 0)
        print(f"Pulse width: {pulse_width} µs")
        time.sleep(2)
        
        # Test 3: Move to right position
        print("Test 3: Moving to right position (180 degrees)")
        pulse_width = angle_to_pulse_width(pi, 180)
        print(f"Pulse width: {pulse_width} µs")
        time.sleep(2)
        
        # Test 4: Move to center position again
        print("Test 4: Moving back to center position (90 degrees)")
        pulse_width = angle_to_pulse_width(pi, 90)
        print(f"Pulse width: {pulse_width} µs")
        time.sleep(2)
        
        # Test 5: Sweep from left to right
        print("Test 5: Sweeping from left to right")
        for angle in range(0, 181, 10):
            pulse_width = angle_to_pulse_width(pi, angle)
            print(f"Angle: {angle}° | Pulse width: {pulse_width} µs", end="\r")
            time.sleep(0.2)
        print("\nSweep complete")
        time.sleep(1)
        
        # Test 6: Sweep from right to left
        print("Test 6: Sweeping from right to left")
        for angle in range(180, -1, -10):
            pulse_width = angle_to_pulse_width(pi, angle)
            print(f"Angle: {angle}° | Pulse width: {pulse_width} µs", end="\r")
            time.sleep(0.2)
        print("\nSweep complete")
        time.sleep(1)
        
        # Test 7: Interactive mode
        print("\nTest 7: Interactive mode")
        print("Enter an angle (0-180) or 'q' to quit:")
        
        while True:
            user_input = input("> ")
            if user_input.lower() == 'q':
                break
            
            try:
                angle = float(user_input)
                if 0 <= angle <= 180:
                    pulse_width = angle_to_pulse_width(pi, angle)
                    print(f"Moving to {angle}° | Pulse width: {pulse_width} µs")
                else:
                    print("Angle must be between 0 and 180 degrees")
            except ValueError:
                print("Invalid input. Enter a number between 0 and 180, or 'q' to quit")
        
        # Return to center position before exiting
        print("Returning to center position")
        angle_to_pulse_width(pi, 90)
        time.sleep(1)
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        # Clean up
        pi.set_servo_pulsewidth(SERVO_PIN, 0)  # Turn off servo
        pi.stop()
        print("Servo test complete")

if __name__ == "__main__":
    main()
