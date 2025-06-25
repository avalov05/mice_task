from adafruit_servokit import ServoKit
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import time
import math
import pygame
import sys
import matplotlib.pyplot as plt
import numpy as np
import io
from PIL import Image
from gpiozero import LED
import os
import datetime
import csv
import signal


#global vars
LED_PIN = 27
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
#=========================================

ADS1115_CHANNEL = 0

def display_debug_text(surface, font, text):
    text = font.render(str(text), True, (255, 0, 0))
    surface.blit(text, (100,100))
def display_debug_text_full(screen, surface, font, text):
    clearScreen(screen, surface)
    display_debug_text(surface, font, text)
    updateBlankScreen(screen, surface)

def testServo(kit):

    print("Enter servo angle:")

    t = 0

    while True:
        #angle = int(input())
        angle = (math.sin(t) + 1) * 90
        print(angle)
        kit.servo[0].angle = angle
        t += 0.01
        time.sleep(0.005)


def updateAnalogGraph(surface, font, x, y, num, sensor_data):

    #=============drawing=============
    text = font.render("Analog " + str(num), True, (COLOR_2))
    surface.blit(text, (x + 420, y))
    text = font.render("5V", True, (COLOR_2))
    surface.blit(text, (x + 5, y + 2))
    text = font.render("0V", True, (COLOR_2))
    surface.blit(text, (x + 5, y + 88))

    pygame.draw.line(surface, COLOR_2, (x, y + 95), (x + 480, y + 95), width = 1)
    pygame.draw.line(surface, COLOR_2, (x + 25, y), (x + 25, y + 100), width = 1)

    space = 90 / 9
    y2 = y + 5
    for loop in range(10):

        pygame.draw.line(surface, COLOR, (x + 23, y2), (x + 27, y2), width = 1)
        pygame.draw.line(surface, COLOR_2, (x + 25, y2), (x + 480, y2), width = 1)
        y2 = y2 + space
        

    pygame.draw.line(surface, COLOR, (x, y + 100), (x+480, y + 100), width = 1)
    #=============drawing=============

    graph_height = 90
    x_g = 25
    for item in sensor_data:

        y_g = item / 5 * graph_height + y
        x_g = x_g + 1
        surface.set_at((x_g, y_g), COLOR)




def updateAnalogGraphs(surface, font, g_data):

    
    updateAnalogGraph(surface, font, 0, 480, 0, g_data["mae4"])
    updateAnalogGraph(surface, font, 0, 580, 1, g_data["colt"])

    



def updateCamera(surface, font):
    pygame.draw.rect(surface, COLOR, (0, 0, 480, 480), width=1)
    working = False
    if working:
        print()
    else:
        x, y = 240, 240
        size = 100
        cam_rect = pygame.Rect(0, 0, size, size * 0.6)
        cam_rect.center = (x, y)
        
        # Camera body
        pygame.draw.rect(surface, COLOR, cam_rect, width = 1, border_radius=8)

        # Lens
        pygame.draw.circle(surface, COLOR, cam_rect.center, 15, width = 1)
        pygame.draw.circle(surface, COLOR, cam_rect.center, 8, width = 1)

        # Flash rectangle (top left of camera body)
        flash_rect = pygame.Rect(0, 0, 15, 10)
        flash_rect.midbottom = (cam_rect.left + 20, cam_rect.top)
        pygame.draw.rect(surface, (255, 0, 0), flash_rect)

        # Display error text
        text = font.render("Camera disconnected", True, (255, 0, 0))
        text_rect = text.get_rect(center=(x, y + 60))
        surface.blit(text, text_rect)

def drawStopButton(surface, font, stop_rect):

    pygame.draw.rect(surface, (200, 0, 0), stop_rect)
    text = font.render("STOP", True, (255, 255, 255))
    text_rect = text.get_rect(center=stop_rect.center)
    surface.blit(text, text_rect)

def updateBlankScreen(screen, surface):
    rotated = pygame.transform.rotate(surface, 90)
    screen.blit(rotated, (10, 0))
    pygame.display.flip()

def clearScreen(screen, surface):
    surface.fill((0,0,0))
    rotated = pygame.transform.rotate(surface, 90)
    screen.blit(rotated, (10, 0))
    pygame.display.flip()
    
def updateScreen(screen, surface, font, g_data, stop_rect, colt):

    surface.fill((0,0,0))

    #all elements
    updateAnalogGraphs(surface, font, g_data)
    updateCamera(surface, font)
    drawStopButton(surface, font, stop_rect)


    #make vertical
    rotated = pygame.transform.rotate(surface, 90)
    screen.blit(rotated, (10, 0))
    pygame.display.flip()

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

def collectData(data, g_data, mae4, colt, start_time, count, disturbance, reward):
    data["time"].append(time.time() - start_time)
    data["mae4"].append(read_voltage(mae4))
    data["colt"].append(read_voltage(colt))
    data["disturbance_angle"].append(disturbance)
    data["reward_delivered"].append(reward)

    #for analog graphs
    if count == 1 :
        count = 1
        g_data["mae4"].append(read_voltage(mae4))
        g_data["colt"].append(read_voltage(colt))
        if len(g_data["mae4"]) >= 455:
            g_data["mae4"] = []
            g_data["colt"] = []
    else:
        count += 1
    
    return count

def line(x, slope, intercept):
    return slope * x + intercept

def calibrateColt(surface, screen, font, colt, kit, num_values):
    clearScreen(screen, surface)
    display_debug_text(surface, font, "calibration in progress...")
    updateBlankScreen(screen, surface)
    starting = MIN_SERVO_ANGLE
    incriment = (MAX_SERVO_ANGLE - MIN_SERVO_ANGLE) / num_values
    x = []
    y = []
    value = starting
    kit.servo[0].angle = value
    x.append(value)
    time.sleep(2)
    CALIBRATION_VALUES["colt"].append(read_voltage(colt))
    y.append(CALIBRATION_VALUES["colt"][0])
    for i in range(1, num_values + 1):
        value = starting + incriment * i
        kit.servo[0].angle = value
        x.append(value)
        time.sleep(0.2)
        CALIBRATION_VALUES["colt"].append(read_voltage(colt))
        y.append(CALIBRATION_VALUES["colt"][i])
    coeffs = np.polyfit(x, y, 1)
    f = np.poly1d(coeffs)
    slope, intercept = coeffs
    x_line = np.linspace(min(x), max(x), 100)
    y_line = line(x_line, slope, intercept)
    fig, ax = plt.subplots(figsize=(4.8, 8), dpi=100)
    ax.plot(x, y, "o", color = "blue")
    plt.plot(x_line, y_line, "-", color="red", label="Best Fit Line")
    ax.set_title("Colt Sensor Calibration")
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format='png')
    plt.close(fig)
    buf.seek(0)
    image = Image.open(buf)
    plot_surface = pygame.image.fromstring(image.tobytes(), image.size, image.mode).convert()
    clearScreen(screen, surface)
    surface.blit(plot_surface, (0, 0))
    display_debug_text(surface, font, "touch anywhere to continue")
    updateBlankScreen(screen, surface)
    waiting = True
    while waiting:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.FINGERDOWN:
                waiting = False
    return f

def updateServoPos(kit, center_angle_calib, desired_position, disturbance = 0):
    needed_angle = desired_position + disturbance + (center_angle_calib - 90)
    if needed_angle < MIN_SERVO_ANGLE:
        kit.servo[0].angle = MIN_SERVO_ANGLE
    elif needed_angle > MAX_SERVO_ANGLE:
        kit.servo[0].angle = MAX_SERVO_ANGLE
    else: kit.servo[0].angle = needed_angle

def calibrateMae4(mae4, screen, surface, font):
    clearScreen(screen, surface)
    display_debug_text(surface, font, "put lever in resting position")
    updateBlankScreen(screen, surface)
    time.sleep(2)
    CALIBRATION_VALUES["mae4"].append(read_voltage(mae4))

def getMae4Angle(mae4):
    return (CALIBRATION_VALUES["mae4"][0] - read_voltage(mae4)) * 360/5

def calibrate_center(kit, screen, surface, font):
    global MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, ADJUSTED_CENTER
    
    kit.servo[0].angle = 90
    clearScreen(screen, surface)
    centering = True
    center_angle = 90

    minus_button_rect = pygame.Rect(0, 700, 240, 100) 
    plus_button_rect = pygame.Rect(240, 700, 240, 100)
    done_button_rect = pygame.Rect(0, 0, 480, 200)

    while centering:
        surface.fill((0,0,0))
        display_debug_text(surface, font, f"Center the servo - Current: {center_angle}°")
    
        pygame.draw.rect(surface, (200, 0, 0), minus_button_rect)
        text = font.render("-1", True, (255, 255, 255))
        text_rect = text.get_rect(center=minus_button_rect.center)
        surface.blit(text, text_rect)
        
        pygame.draw.rect(surface, (0, 200, 0), plus_button_rect)
        text = font.render("+1", True, (255, 255, 255))
        text_rect = text.get_rect(center=plus_button_rect.center)
        surface.blit(text, text_rect)

        pygame.draw.rect(surface, (0, 0, 200), done_button_rect)
        text = font.render("Done", True, (255, 255, 255))
        text_rect = text.get_rect(center=done_button_rect.center)
        surface.blit(text, text_rect)
        
        updateBlankScreen(screen, surface)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.FINGERDOWN:
                touch_x = event.y * 480 
                touch_y = event.x * 800
                
                if minus_button_rect.collidepoint(touch_x, touch_y):
                    center_angle = center_angle - 1
                    ADJUSTED_CENTER -= 1
                    MIN_SERVO_ANGLE -= 1
                    MAX_SERVO_ANGLE -= 1
                    kit.servo[0].angle = center_angle
                elif plus_button_rect.collidepoint(touch_x, touch_y):
                    center_angle = center_angle + 1
                    ADJUSTED_CENTER += 1
                    MIN_SERVO_ANGLE += 1
                    MAX_SERVO_ANGLE += 1
                    kit.servo[0].angle = center_angle
                elif done_button_rect.collidepoint(touch_x, touch_y):
                    centering = False

        time.sleep(0.01)

    return center_angle

#shows the graphs that show the task parameters
def taskInit(screen, surface, font):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(4.7, 7.7), dpi=100, constrained_layout=True)
    fig.tight_layout(pad=1.0)
    fig.subplots_adjust(hspace=0.3)
    
    #graph 1
    ax1.set_title('FBF')
    ax1.set_xlabel('Lever Angle (deg)')
    ax1.set_ylabel('Spout Pos (deg)')
    ax1.grid(True)

    min_servo_angle_adjusted = (MIN_SERVO_ANGLE - 90) * GEAR_RATIO
    max_servo_angle_adjusted = (MAX_SERVO_ANGLE - 90) * GEAR_RATIO
    
    x = np.linspace(MIN_LEVER_ANGLE, MAX_LEVER_ANGLE, 100)
    y = (FBF(x) - 90) * GEAR_RATIO
    ax1.plot(x, y, 'b-', linewidth=1)
    ax1.set_ylim(min_servo_angle_adjusted, max_servo_angle_adjusted )
    
    #reward zone
    reward_zone_min = (ADJUSTED_CENTER - REWARD_ZONE_SIZE / 2 - ADJUSTED_CENTER) * GEAR_RATIO
    reward_zone_max = (ADJUSTED_CENTER + REWARD_ZONE_SIZE / 2 - ADJUSTED_CENTER) * GEAR_RATIO
    ax1.axhspan(reward_zone_min, reward_zone_max, alpha=0.3, color='green', label='Reward Zone')
    ax1.legend()
    ax2.axhspan(reward_zone_min, reward_zone_max, alpha=0.3, color='green', label='Reward Zone')
    ax2.legend()
    
    #graph 2
    ax2.set_title('Disturbance')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Spout Pos (deg)')
    ax2.grid(True)
    ax2.set_ylim(min_servo_angle_adjusted, max_servo_angle_adjusted )
    
    x = np.linspace(0, 10, 100)
    y = (get_disturbance(x)) * GEAR_RATIO
    ax2.plot(x, y, 'r-', linewidth=1)
    
    buf = io.BytesIO()
    fig.savefig(buf, format='png', bbox_inches='tight', dpi=100)
    plt.close(fig)
    buf.seek(0)
    image = Image.open(buf)
    plot_surface = pygame.image.fromstring(image.tobytes(), image.size, image.mode).convert()
    
    clearScreen(screen, surface)
    surface.blit(plot_surface, (0, 0))
    display_debug_text(surface, font, "touch anywhere to continue")
    updateBlankScreen(screen, surface)

    waiting = True
    while waiting:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.FINGERDOWN:
                waiting = False

def FBF(lever_angle):
    global LEVER_SENSITIVITY
    x_1 = (MAX_LEVER_ANGLE - MIN_LEVER_ANGLE) / 2 + MIN_LEVER_ANGLE
    y_1 = 90
    x_2 = MIN_LEVER_ANGLE
    y_2 = MIN_SERVO_ANGLE
    m = (y_1 - y_2)/(x_1 - x_2)
    b = y_1 - m * x_1
    m_adjusted = m * LEVER_SENSITIVITY
    b = y_1 - m_adjusted * x_1
    return m_adjusted * lever_angle + b

def get_disturbance(time):
    if DISTURBANCE_TYPE == "NONE":
        return time * 0
    elif DISTURBANCE_TYPE == "SIN":
        return math.sin(time)
    
def check_reward(colt, colt_fun):
    m, b = colt_fun.coeffs
    x = (read_voltage(colt) - b) / m
    if REWARD_ZONE_TYPE == "zone":
        if x > ADJUSTED_CENTER - REWARD_ZONE_SIZE / 2 and x < ADJUSTED_CENTER + REWARD_ZONE_SIZE / 2:
            return True
        else: 
            return False
    elif REWARD_ZONE_TYPE == "press":
        if x > REWARD_ZONE_DISTANCE + 90:
            return True
        else: 
            return False
    
    
def dispense_reward(led, duration=0.007):
    
    led.on()
    time.sleep(duration)
    led.off()

def save_data_to_file(data, data_filename):
    with open(data_filename, 'w', newline='') as data_file:
        data_writer = csv.writer(data_file, delimiter='\t')
        data_writer.writerow([
            "Time (s)",
            "Servo Angle (degrees)",
            "Lever Angle (degrees)",
            "Disturbance Angle",
            "Reward Delivered (0/1)"
        ])
        
        for i in range(len(data["time"])):
            data_writer.writerow([
                data["time"][i],
                f"{data['colt'][i]:.1f}",
                f"{data['mae4'][i]:.1f}",
                f"{data['disturbance_angle'][i]:.1f}",
                data['reward_delivered'][i]
            ])
    print(f"Data saved to: {data_filename}")

def signal_handler(sig, frame):
    print("\nCtrl+C detected. Saving data...")
    save_data_to_file(data, data_filename)
    print("Data saved successfully. Exiting...")
    sys.exit(0)

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

    data = {"time":[], "mae4":[], "colt":[], "disturbance_angle":[], "reward_delivered":[]}
    
    signal.signal(signal.SIGINT, signal_handler)

    #initialize
    kit = ServoKit(channels=16)
    led = LED(LED_PIN)
    pygame.init()
    screen = pygame.display.set_mode((480, 800))
    virtual_surface = pygame.Surface((480, 800))
    font = pygame.font.SysFont(None, 20)
    count = 1
    stop_button_rect = pygame.Rect(0, 700, 480, 100)
    # Initialize I2C bus and ADS1115
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    mae4 = ADS.ADS1115(i2c, address=0x48)
    colt = ADS.ADS1115(i2c, address=0x49)
    mae4.gain = 1
    colt.gain = 1


    g_data = {"mae4":[], "colt":[]}

    start_time = time.time()
    last_reward = 0
    hold_start = start_time
    in_reward_zone = False  #track if we are currently in reward zone

    t = 0

    center_calibration_angle = calibrate_center(kit, screen, virtual_surface, font)
    colt_fun = calibrateColt(virtual_surface, screen, font, colt, kit, 20)
    taskInit(screen, virtual_surface, font)
    calibrateMae4(mae4, screen, virtual_surface, font)
    pygame.quit()

    start_time = time.time()
    
    
    
    while True:
        #virtual_surface.fill((0,0,0)) commented for optimization


        #complete lever to spout conversion
        lever_angle = getMae4Angle(mae4)
        FBF_angle = FBF(lever_angle)
        elapsed_time = time.time() - start_time
        disturbance_angle = get_disturbance(elapsed_time)
        updateServoPos(kit, center_calibration_angle, FBF_angle, disturbance_angle)
        
        #check if reward should be given
        current_time = time.time() - start_time
        if check_reward(colt, colt_fun):
            if not in_reward_zone:
                hold_start = current_time
                in_reward_zone = True
            
            if current_time - last_reward >= MIN_TIME_BETWEEN_REWARDS:
                if current_time - hold_start >= MIN_TIME_BETWEEN_REWARDS:
                    dispense_reward(led)
                    last_reward = current_time
                    hold_start = current_time
                    reward_delivered = 1
                else:
                    reward_delivered = 0
            else:
                reward_delivered = 0
        else:
            in_reward_zone = False
            reward_delivered = 0


        m, b = colt_fun.coeffs
        servo_angle = (read_voltage(colt) - b) / m
        count = collectData(data, g_data, lever_angle, servo_angle, start_time, count,disturbance_angle, reward_delivered)
        #updateScreen(screen, virtual_surface, font, g_data, stop_button_rect, colt)  commented for optimization

        #display_debug_text_full(screen, virtual_surface, font, getMae4Angle(mae4))

        

        # data_writer.writerow([
        #     current_time,
        #     f"{servo_angle:.1f}",
        #     f"{lever_angle:.1f}",
        #     f"{disturbance_angle:.1f}",   commented for optimization
        #     reward_delivered
        #     ])
        # data_file.flush()
        

    
        # for event in pygame.event.get():
        #     if event.type == pygame.QUIT:
        #         pygame.quit()
        #         sys.exit()

        #     elif event.type == pygame.FINGERDOWN:
        #         touch_x = event.y * 480 #rotation
        #         touch_y = event.x * 800 
        #         if stop_button_rect.collidepoint(touch_x, touch_y):
        #             pygame.quit()
        #             sys.exit()
            
        #     elif event.type == pygame.MOUSEBUTTONDOWN:
        #         if stop_button_rect.collidepoint(event.pos):
        #             pygame.quit()
        #             sys.exit()

        # time.sleep(0.01)

main()