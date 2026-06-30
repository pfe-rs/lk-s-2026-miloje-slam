import os
import time
from rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'
SAVE_DIR = 'scans/real'

def get_next_filename(directory):
    """Finds the next available filename to prevent overwriting."""
    if not os.path.exists(directory):
        os.makedirs(directory)
    
    counter = 1
    while True:
        filename = f"real_lidar_scan_{counter}.csv"
        full_path = os.path.join(directory, filename)
        if not os.path.exists(full_path):
            return full_path
        counter += 1

def main():
    lidar = RPLidar(PORT_NAME)
    
    print('RPLidar Information:', lidar.get_info())
    print('Health status:', lidar.get_health())
    print('Waiting for motor to stabilize...')
    time.sleep(2)  # Give the motor an extra moment to reach full speed
    
    raw_points = []
    start_collecting = False
    
    print('Searching for the $0^\circ$ start flag...')
    
    try:
        # iter_measurements yields: (new_scan_flag, quality, angle, distance)
        for new_scan, _, angle, distance in lidar.iter_measures():
            
            # 1. Wait for the hardware to signal a brand new 360 rotation loop
            if new_scan:
                if not start_collecting:
                    # Found the start of our clean sweep!
                    start_collecting = True
                    print('Start flag found! Collecting one full rotation...')
                    # Capture this initial boundary point
                    if distance > 0: 
                        raw_points.append((angle, distance))
                    continue
                else:
                    # The flag has triggered a second time. We have completed a full 360° turn.
                    print('End flag reached. Processing sweep data...')
                    break
            
            # 2. Collect data while inside our target rotation window
            if start_collecting:
                # Filter out zero/failed readings (where distance is 0)
                if distance > 0:
                    raw_points.append((angle, distance))
                    
        # 3. Clean and save the data
        if raw_points:
            # Sort points sequentially by angle (0 -> 360) so the CSV is organized perfectly
            raw_points.sort(key=lambda x: x[0])
            
            file_path = get_next_filename(SAVE_DIR)
            with open(file_path, 'w') as f:
                for angle, distance in raw_points:
                    # Optional: If you need meters instead of mm, use (distance / 1000)
                    f.write(f"{distance:.18e},{angle:.18e}\n")
            
            print(f"Successfully saved {len(raw_points)} points to {file_path}")
        else:
            print("Error: No valid points were captured.")

    except Exception as e:
        print(f"An error occurred: {e}")
        
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print('Lidar disconnected safely.')

if __name__ == '__main__':
    main()