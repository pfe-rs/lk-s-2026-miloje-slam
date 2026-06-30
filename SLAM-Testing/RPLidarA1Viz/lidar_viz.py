# import threading
# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# from rplidar import RPLidar

# PORT_NAME = '/dev/ttyUSB0'   # USB port lidara
# MAX_DISTANCE = 10000         # 10m maksimalna udaljenost

# lidar = RPLidar(PORT_NAME)

# # Threadovanje
# latest_scan = []
# data_lock = threading.Lock()
# is_running = True

# def lidar_background_worker():
#     # Kontinualno citanje sa lidara
#     global latest_scan, is_running
#     try:
#         for scan in lidar.iter_scans():
#             if not is_running:
#                 break
#             with data_lock:
#                 latest_scan = scan
#     except Exception as e:
#         print(f"Greska: {e}")

# # Pokretanje citaca
# worker_thread = threading.Thread(target=lidar_background_worker, daemon=True)
# worker_thread.start()

# # Matplotlib plot
# fig = plt.figure(figsize=(8, 8))
# ax = fig.add_subplot(111, projection='polar')
# ax.set_ylim(0, MAX_DISTANCE)
# line, = ax.plot([], [], 'r.', markersize=2)

# def update(frame):
#     # Apdejt GUI-a
#     global latest_scan
    
#     # Uzima sliku trenutnih podataka
#     with data_lock:
#         current_scan = list(latest_scan)
        
#     if current_scan:
#         angles = []
#         distances = []
#         for _, angle, distance in current_scan:
#             angles.append(np.radians(angle))
#             distances.append(distance)
        
#         # Apdejt
#         line.set_data(angles, distances)
        
#     return line,

# try:
#     print("Pokrenuto")
#     # Animacija plota ~20fps
#     ani = FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
#     plt.show()

# except KeyboardInterrupt:
#     print("Kraj")

# finally:
#     # Gasenje threadovanja
#     is_running = False
#     print("Stopiranje lidara")
#     lidar.stop()
#     lidar.disconnect()
#     print("Pocisceno")


import threading
# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# from rplidar import RPLidar

# PORT_NAME = '/dev/ttyUSB0'   # USB port lidara
# MAX_DISTANCE = 10000         # 10m maksimalna udaljenost

# lidar = RPLidar(PORT_NAME)

# # Threadovanje
# latest_scan = []
# data_lock = threading.Lock()
# is_running = True

# def lidar_background_worker():
#     # Kontinualno citanje sa lidara
#     global latest_scan, is_running
#     try:
#         for scan in lidar.iter_scans():
#             if not is_running:
#                 break
#             with data_lock:
#                 latest_scan = scan
#     except Exception as e:
#         print(f"Greska: {e}")

# # Pokretanje citaca
# worker_thread = threading.Thread(target=lidar_background_worker, daemon=True)
# worker_thread.start()

# # Matplotlib plot
# fig = plt.figure(figsize=(8, 8))
# ax = fig.add_subplot(111, projection='polar')
# ax.set_ylim(0, MAX_DISTANCE)
# line, = ax.plot([], [], 'r.', markersize=2)

# def update(frame):
#     # Apdejt GUI-a
#     global latest_scan
    
#     # Uzima sliku trenutnih podataka
#     with data_lock:
#         current_scan = list(latest_scan)
        
#     if current_scan:
#         angles = []
#         distances = []
#         for _, angle, distance in current_scan:
#             angles.append(np.radians(angle))
#             distances.append(distance)
        
#         # Apdejt
#         line.set_data(angles, distances)
        
#     return line,

# try:
#     print("Pokrenuto")
#     # Animacija plota ~20fps
#     ani = FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
#     plt.show()

# except KeyboardInterrupt:
#     print("Kraj")

# finally:
#     # Gasenje threadovanja
#     is_running = False
#     print("Stopiranje lidara")
#     lidar.stop()
#     lidar.disconnect()
#     print("Pocisceno")




from rplidar import RPLidar
import time

PORT_NAME = '/dev/ttyUSB0'

def main():
    lidar = RPLidar(PORT_NAME)
    
    # Get device information
    info = lidar.get_info()
    print('RPLidar Information:', info)
    
    # Get device health status
    health = lidar.get_health()
    print('Health status:', health)
    
    # Start scanning and read data
    print('Starting scan, press Ctrl+C to stop...')
    time.sleep(1)
    
    try:
        for scan in lidar.iter_scans():
            for (_, angle, distance) in scan:
                print(f'Angle: {angle}°, Distance: {distance} mm')
    except KeyboardInterrupt:
        print('Scan stopped.')
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

if __name__ == '__main__':
    main()