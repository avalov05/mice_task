import socket
import json
import matplotlib.pyplot as plt
from collections import defaultdict

HOST = '0.0.0.0'      # Listen on all interfaces
PORT = 12345

# Data storage
all_data = defaultdict(dict)

# Setup plotting
plt.ion()  # Enable interactive mode
fig, ax = plt.subplots(figsize=(10, 6))
lines = {}

def update_plot():
    """Update the plot with the latest data"""
    ax.clear()
    
    # Set dark mode style
    plt.style.use('dark_background')
    fig.patch.set_facecolor('#121212')
    ax.set_facecolor('#1E1E1E')
    
    # Plot each data series
    times = sorted(all_data.keys())
    if not times:
        return
        
    # Extract data series
    disturbance_angles = [all_data[t].get("disturbance_angle", None) for t in times]
    hall_sensor_angles = [all_data[t].get("hall_sensor_angle", None) for t in times]
    final_angles = [all_data[t].get("final_angle", None) for t in times]
    in_zone = [all_data[t].get("in_zone", False) for t in times]
    
    # Find reward delivery points - these are points where a reward was actually delivered
    # In the task.py, a reward is delivered after being in the zone for time_for_reward (1 second)
    reward_times = []
    in_zone_duration = 0
    prev_time = None
    
    for i, t in enumerate(times):
        if in_zone[i]:
            # Calculate how long we've been in the zone
            if prev_time is not None:
                in_zone_duration += (t - prev_time)
            
            # Check if we've been in the zone for at least 1 second
            if in_zone_duration >= 1.0:
                reward_times.append(t)
                # Reset the duration after a reward is delivered
                in_zone_duration = 0
        else:
            # Reset duration when we leave the zone
            in_zone_duration = 0
        
        prev_time = t
    
    # Plot data with enhanced styling
    ax.plot(times, disturbance_angles, color='#FF5555', linewidth=2, label='Disturbance Angle')
    ax.plot(times, hall_sensor_angles, color='#55FF55', linewidth=2, label='Hall Sensor Angle')
    ax.plot(times, final_angles, color='#5555FF', linewidth=2, label='Final Angle')
    
    # Highlight delivery zone periods
    zone_starts = []
    zone_ends = []
    in_zone_now = False
    
    for i, t in enumerate(times):
        if in_zone[i] and not in_zone_now:
            zone_starts.append(t)
            in_zone_now = True
        elif not in_zone[i] and in_zone_now:
            zone_ends.append(t)
            in_zone_now = False
    
    # Add the last zone end if needed
    if in_zone_now and zone_starts:
        zone_ends.append(times[-1])
    
    # Draw the zone periods
    for start, end in zip(zone_starts, zone_ends):
        ax.axvspan(start, end, alpha=0.2, color='#00FF00')
    
    # Mark reward points with stars
    for rt in reward_times:
        ax.plot(rt, 90, marker='*', markersize=15, color='gold')
    
    # Add target zone indicator
    ax.axhspan(90 - 10, 90 + 10, alpha=0.1, color='#FFFF00', linestyle='--')
    ax.axhline(90, color='#FFFF00', linestyle='--', alpha=0.5)
    
    # Add labels and legend with custom styling
    ax.set_xlabel('Time (s)', color='white', fontsize=12)
    ax.set_ylabel('Angle (degrees)', color='white', fontsize=12)
    ax.set_title('Angle Tracking and Reward Delivery', color='white', fontsize=14, fontweight='bold')
    
    # Customize legend
    legend = ax.legend(loc='upper right', fancybox=True, framealpha=0.7)
    legend.get_frame().set_facecolor('#2A2A2A')
    for text in legend.get_texts():
        text.set_color('white')
    
    # Customize grid
    ax.grid(True, linestyle='--', alpha=0.3)
    
    # Set y-axis limits with some padding
    ax.set_ylim(0, 180)
    
    # Customize ticks
    ax.tick_params(colors='white')
    
    # Add annotation for rewards
    if reward_times:
        ax.annotate(f'Rewards: {len(reward_times)}', 
                   xy=(0.02, 0.96), 
                   xycoords='axes fraction',
                   color='gold',
                   fontsize=12,
                   bbox=dict(boxstyle="round,pad=0.3", facecolor='#2A2A2A', alpha=0.7))
    
    plt.tight_layout()
    plt.draw()
    plt.pause(0.01)

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)

print(f"Listening on {PORT}...")
conn, addr = server.accept()
print(f"Connected by {addr}")

# Buffer for incomplete data
buffer = ""

try:
    while True:
        data = conn.recv(16384)  # Increased buffer size significantly
        if not data:
            break
        
        # Add received data to buffer
        buffer += data.decode('utf-8', errors='replace')
        
        # Try to find complete JSON objects
        try:
            # Check if we have a complete JSON object
            parsed = json.loads(buffer)
            print(f"Received complete data with {len(parsed)} points")
            
            # Update our data store
            for timestamp, values in parsed.items():
                all_data[float(timestamp)] = values
            
            # Clear buffer after successful parsing
            buffer = ""
            
            # Update the plot
            update_plot()
            
        except json.JSONDecodeError:
            # If we can't parse the JSON, it might be incomplete
            # Keep the buffer and wait for more data
            print(f"Received partial data, waiting for more... (Buffer size: {len(buffer)})")
            
except Exception as e:
    print(f"Error: {e}")
finally:
    conn.close()
    server.close()
    plt.ioff()  # Turn off interactive mode
    plt.show()  # Keep the plot window open