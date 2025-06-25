import time
import struct
import busio
import board
import adafruit_ads1x15.ads1115 as ADS
import matplotlib.pyplot as plt
import math
from adafruit_servokit import ServoKit

# Initialize
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
ads1 = ADS.ADS1115(i2c, address=0x48)
ads2 = ADS.ADS1115(i2c, address=0x49)
kit = ServoKit(channels=16)

ads1.gain = 1
ads2.gain = 1

def read_analog(ads):
    # Trigger single-shot conversion on channel 0
    config = 0x8580  # Single-shot, channel 0 (AIN0 vs GND), 860 SPS
    ads._write_register(0x01, config)
    
    # Use a fixed delay - much faster than polling
    # At 860 SPS, conversion takes ~1.2ms, but we can use shorter delay
    #time.sleep(0.002)  # 2ms delay - should be enough for conversion
    
    # Read the conversion result
    raw = ads._read_register(0x00)
    if raw > 0x7FFF:
        raw -= 0x10000
    return raw

def raw_to_voltage(raw_value, gain=1):
    """
    Convert raw ADC value to voltage
    gain: 1 = ±4.096V, 2 = ±2.048V, 4 = ±1.024V, 8 = ±0.512V, 16 = ±0.256V
    """
    # Full scale range for 16-bit ADC (±32767)
    full_scale = 32767
    
    # Voltage ranges for different gains
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

# Main loop
start_time = time.time()
data = {"time":[], "ch1":[], "ch2":[], "ch1_v":[], "ch2_v":[]}
count = 0

while time.time() - start_time < 10:
    current_time = time.time() - start_time
    
    # Read from channel 0 on both ADCs for maximum speed
    value1 = read_analog(ads1)  # AIN0 on first ADC
    value2 = read_analog(ads2)  # AIN0 on second ADC
    
    # Convert to voltage
    voltage1 = raw_to_voltage(value1, gain=1)
    voltage2 = raw_to_voltage(value2, gain=1)
    
    servo_angle = 90 + 90 * math.sin(current_time * 2)
    kit.servo[0].angle = servo_angle
    
    data["time"].append(current_time)
    data["ch1"].append(value1)
    data["ch2"].append(value2)
    data["ch1_v"].append(voltage1)
    data["ch2_v"].append(voltage2)
    count += 1
    
    if count % 100 == 0:
        print(f"Ch1: {value1} ({voltage1:.3f}V), Ch2: {value2} ({voltage2:.3f}V)")

print(f"Samples: {count}, Hz: {count/10:.1f}")

# Plot
plt.figure(figsize=(12, 10))
plt.subplot(2, 1, 1)
plt.plot(data["time"], data["ch1"])
plt.ylabel('ADC1 Ch0 (0x48)')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(data["time"], data["ch2"])
plt.ylabel('ADC2 Ch0 (0x49)')
plt.grid(True)

plt.tight_layout()
plt.savefig('dual_adc_plot.png', dpi=150, bbox_inches='tight')
plt.close()
print("Plot saved as 'dual_adc_plot.png'")
