#!/usr/bin/env python3
"""
Test script to verify coordinate conversion between servo and lever angles
"""

# Copy the relevant functions from taskV2.py
ADJUSTED_CENTER = 90
GEAR_RATIO = 1/5
MAX_SERVO_ANGLE = 135
MIN_SERVO_ANGLE = 45
MAX_LEVER_ANGLE = 45
MIN_LEVER_ANGLE = 0
LEVER_SENSITIVITY = 1
REWARD_ZONE_SIZE = 40

def FBF(lever_angle):
    """Forward transformation: lever angle -> servo angle"""
    global LEVER_SENSITIVITY
    x_1 = (MAX_LEVER_ANGLE - MIN_LEVER_ANGLE) / 2 + MIN_LEVER_ANGLE
    y_1 = ADJUSTED_CENTER
    x_2 = MIN_LEVER_ANGLE
    y_2 = MIN_SERVO_ANGLE
    m = (y_1 - y_2)/(x_1 - x_2)
    b = y_1 - m * x_1
    m_adjusted = m * LEVER_SENSITIVITY 
    b = y_1 - m_adjusted * x_1
    return m_adjusted * lever_angle + b

def servo_to_lever_angle(servo_angle):
    """Inverse transformation: servo angle -> lever angle"""
    global LEVER_SENSITIVITY
    x_1 = (MAX_LEVER_ANGLE - MIN_LEVER_ANGLE) / 2 + MIN_LEVER_ANGLE
    y_1 = ADJUSTED_CENTER
    x_2 = MIN_LEVER_ANGLE
    y_2 = MIN_SERVO_ANGLE
    m = (y_1 - y_2)/(x_1 - x_2)
    b = y_1 - m * x_1
    m_adjusted = m * LEVER_SENSITIVITY 
    b = y_1 - m_adjusted * x_1
    
    # Inverse of FBF: lever_angle = (servo_angle - b) / m_adjusted
    return (servo_angle - b) / m_adjusted

def test_conversion():
    """Test the coordinate conversion functions"""
    print("Testing coordinate conversion between servo and lever angles")
    print("=" * 60)
    
    # Test reward zone boundaries
    servo_zone_min = ADJUSTED_CENTER - REWARD_ZONE_SIZE / 2
    servo_zone_max = ADJUSTED_CENTER + REWARD_ZONE_SIZE / 2
    
    print(f"Reward zone in servo space: {servo_zone_min:.1f}° to {servo_zone_max:.1f}°")
    
    lever_zone_min = servo_to_lever_angle(servo_zone_min)
    lever_zone_max = servo_to_lever_angle(servo_zone_max)
    
    print(f"Reward zone in lever space: {lever_zone_min:.1f}° to {lever_zone_max:.1f}°")
    print()
    
    # Test round-trip conversion
    print("Testing round-trip conversion:")
    test_lever_angles = [0, 10, 20, 30, 40, 45]
    
    for lever_angle in test_lever_angles:
        servo_angle = FBF(lever_angle)
        lever_angle_back = servo_to_lever_angle(servo_angle)
        
        print(f"Lever: {lever_angle:2.0f}° -> Servo: {servo_angle:5.1f}° -> Lever: {lever_angle_back:5.1f}°")
    
    print()
    
    # Test some servo angles
    print("Testing servo to lever conversion:")
    test_servo_angles = [45, 60, 90, 120, 135]
    
    for servo_angle in test_servo_angles:
        lever_angle = servo_to_lever_angle(servo_angle)
        servo_angle_back = FBF(lever_angle)
        
        print(f"Servo: {servo_angle:3.0f}° -> Lever: {lever_angle:5.1f}° -> Servo: {servo_angle_back:5.1f}°")

if __name__ == "__main__":
    test_conversion() 