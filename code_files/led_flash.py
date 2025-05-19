import pigpio
import time
import signal
import sys

# GPIO pin for the LED
LED_PIN = 27
# PWM frequency in Hz
PWM_FREQ = 1000
# Maximum duty cycle (100%)
MAX_DUTY = 1000000

def signal_handler(sig, frame):
    """
    Handle Ctrl+C by turning off the LED before exiting
    """
    print("\nTurning off LED and exiting...")
    pi.set_mode(LED_PIN, pigpio.OUTPUT)
    pi.write(LED_PIN, 0)  # Turn off LED
    pi.stop()
    sys.exit(0)

def deliver_reward():
    """
    Deliver reward by setting maximum voltage to the pin
    """
    # Set PWM frequency and maximum duty cycle
    pi.hardware_PWM(LED_PIN, PWM_FREQ, MAX_DUTY)
    time.sleep(0.1)  # Keep reward on for 100ms
    pi.hardware_PWM(LED_PIN, PWM_FREQ, 0)  # Turn off

def main():
    # Set up signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize pigpio
    global pi
    pi = pigpio.pi()
    if not pi.connected:
        print("Failed to connect to pigpio daemon")
        exit(1)

    # Set up LED pin as output
    pi.set_mode(LED_PIN, pigpio.OUTPUT)
    
    print("Reward system ready. Press Ctrl+C to stop.")
    
    try:
        while True:
            # Deliver reward
            deliver_reward()
            # Wait 2 seconds between rewards
            time.sleep(2)
            
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main() 