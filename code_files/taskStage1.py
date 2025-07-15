from adafruit_servokit import ServoKit
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
import time
import sys
from gpiozero import LED
import os
import datetime
import csv
import signal
import random
import threading


#global vars
LED_PIN = 27
TRIAL_LED_PIN = 22
COLOR = (200, 200, 200) #general 
COLOR_2 = (100, 100, 100)
CALIBRATION_VALUES = {"mae4":[], "colt":[]}
ADJUSTED_CENTER = 90
GEAR_RATIO = 1/5

#=============TASK PARAMETERS=============
MAX_SERVO_ANGLE = 135
MIN_SERVO_ANGLE = 45
MAX_LEVER_ANGLE = 45
MIN_LEVER_ANGLE = 0

LEVER_SENSITIVITY = 1
REWARD_ZONE_SIZE = 40 #overall
REWARD_ZONE_DISTANCE = 10 #for exponential
REWARD_ZONE_TYPE = "zone" #zone, press
DISTURBANCE_TYPE = "NONE"
MIN_TIME_BETWEEN_REWARDS = .5 #seconds

ENABLE_TRIALS = True
TRIAL_LENGTH = 60
TRIAL_DISTANCE_RANGE = (20, 60)
#=========================================

ADS1115_CHANNEL = 0

def signal_handler(sig, frame):
    print("\nCtrl+C detected. Saving data...")
    save_data_to_file(data, data_filename)
    print("Data saved successfully. Exiting...")
    sys.exit(0)

def dispense_reward(led, duration=0.010):
    def pulse():
        led.on()
        data["reward_delivered"][-1] = 1
        time.sleep(duration)
        led.off()
        print("Reward given at ", time.time())

    threading.Thread(target=pulse, daemon=True).start()

def save_data_to_file(data, data_filename):
    with open(data_filename, 'w', newline='') as data_file:
        data_writer = csv.writer(data_file, delimiter='\t')
        data_writer.writerow([
            "Time (s)",
            "Servo Angle (degrees)",
            "Lever Angle (degrees)",
            "Disturbance Angle",
            "Reward Delivered (0/1)",
            "Trial Number/Status"
        ])
        
        for i in range(len(data["time"])):
            data_writer.writerow([
                data["time"][i],
                f"{data['colt'][i]:.1f}",
                f"{data['mae4'][i]:.1f}",
                f"{data['disturbance_angle'][i]:.1f}",
                data['reward_delivered'][i],
                data["trial"][i]
            ])
    print(f"Data saved to: {data_filename}")

def updateServoPos(kit, desired_position, disturbance = 0):
    needed_angle = desired_position + disturbance + (ADJUSTED_CENTER - 90)
    if needed_angle < MIN_SERVO_ANGLE:
        kit.servo[0].angle = MIN_SERVO_ANGLE
    elif needed_angle > MAX_SERVO_ANGLE:
        kit.servo[0].angle = MAX_SERVO_ANGLE
    else: kit.servo[0].angle = needed_angle

def calibrateMae4(mae4):
    for _ in range(100):
        read_voltage(mae4)
    CALIBRATION_VALUES["mae4"].append(read_voltage(mae4))
    print(CALIBRATION_VALUES["mae4"][0])

def getMae4Angle(mae4):
    return (CALIBRATION_VALUES["mae4"][0] - read_voltage(mae4)) * 360/5

def read_analog(ads):
    # Trigger single-shot conversion on channel 0
    config = 0x8580  # Single-shot, channel 0 (AIN0 vs GND), 860 SPS
    ads._write_register(0x01, config)
    
    # Read the conversion result
    raw = ads._read_register(0x00)
    if raw > 0x7FFF:
        raw -= 0x10000
    return raw

def raw_to_voltage(raw_value, gain=1):
    # Full scale range for 16-bit ADC (±32767)
    full_scale = 32767

    voltage_ranges = {
        1: 4.096,   # ±4.096V
        2: 2.048,   # ±2.048V  
        4: 1.024,   # ±1.024V
        8: 0.512,   # ±0.512V
        16: 0.256   # ±0.256V
    }
    
    voltage_range = voltage_ranges[gain]
    voltage = (raw_value / full_scale) * voltage_range
    return voltage

def read_voltage(ads):
    raw = read_analog(ads)
    voltage = raw_to_voltage(raw)
    return voltage

def read_voltage(ads):
    raw = read_analog(ads)
    voltage = raw_to_voltage(raw)
    return voltage

def collectData(data, mae4, colt, start_time, disturbance, trial = False):
    data["time"].append(time.time() - start_time)
    data["mae4"].append(mae4)
    data["colt"].append(colt)
    data["disturbance_angle"].append(disturbance)
    data["reward_delivered"].append(0)
    data["trial"].append(trial)



def stage_one(kit, trial_led, led, mae4, start_time):

    stop_threads = threading.Event()
    reward_active = False
    trial = True
    trial_data = 0

    #==========Parameters==========
    reward_block_duration = 10
    reward_distance = 1
    lever_displacement = 2 #to trigger reward block
    trial_length = 50
    trial_distance_range = (45, 60)
    data_collection_interval = 0.01  #10 ms interval for data collection
    #==============================

    def combined_data_and_reward_loop(mae4, led):
        nonlocal trial
        nonlocal reward_active
        nonlocal trial_data
        last_data_time = time.time()
        last_reward_time = time.time()
        
        while not stop_threads.is_set():
            current_time = time.time()
            
            if current_time - last_data_time >= data_collection_interval:
                Hz = 1/(current_time - last_data_time)
                print("                 ", Hz, end='\r')
                last_data_time = current_time
                
                lever_angle = getMae4Angle(mae4)
                collectData(data, lever_angle, 0, start_time, 0, trial_data)
            
                if lever_angle >= lever_displacement and (current_time - last_reward_time) >= reward_distance and trial:
                    dispense_reward(led)
                    last_reward_time = current_time
            
            time.sleep(0.001) 
    
    def trial_loop(led, trial_led):
        nonlocal trial
        nonlocal trial_data
        
        trial_count = 0
        trial_data = 0

        while not stop_threads.is_set():
            if trial:
                trial_count += 1
                trial_data = trial_count
                trial_led.on()
                dispense_reward(led, duration=0.03)
                time.sleep(trial_length)
                trial = False
            else:
                trial_data = 0
                trial_led.off()
                time.sleep(random.randint(trial_distance_range[0], trial_distance_range[1]))
                trial = True
        time.sleep(0.01)

    try:

        updateServoPos(kit, ADJUSTED_CENTER)

        combined_thread = threading.Thread(target=combined_data_and_reward_loop, args=(mae4, led), daemon=True)
        trial_thread = threading.Thread(target=trial_loop, args=(led, trial_led), daemon=True)

        combined_thread.start()
        trial_thread.start()

        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        stop_threads.set()
        combined_thread.join()
        trial_thread.join()
        
        print("Program stopped cleanly.")

def main():
    data_dir = os.path.join(os.getcwd(), "task_data")
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)

    print("\nPlease enter the mice ID (e.g., D147M51):")
    mice_id = input().strip()

    while not mice_id:
        print("Mice ID cannot be empty. Please enter a valid mice ID:")
        mice_id = input().strip()

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    
    global data, data_filename
    data_filename = os.path.join(data_dir, f"{mice_id}_{timestamp}.txt")

    print(f"Data will be saved to: {data_filename}")

    data = {"time":[], "mae4":[], "colt":[], "disturbance_angle":[], "reward_delivered":[], "trial":[]}
    
    signal.signal(signal.SIGINT, signal_handler)

    #initialize
    kit = ServoKit(channels=16)
    led = LED(LED_PIN)
    trial_led = LED(TRIAL_LED_PIN)

    # Initialize I2C bus and ADS1115
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    mae4 = ADS.ADS1115(i2c, address=0x48)
    colt = ADS.ADS1115(i2c, address=0x49)
    mae4.gain = 1
    colt.gain = 1

    print("Put lever in resting position and press Enter...")
    input()
    calibrateMae4(mae4)

    start_time = time.time()

    collectData(data, 0, 0, start_time, 0, 0)



    stage_one(kit, trial_led, led, mae4, start_time)

main()