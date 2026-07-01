import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from rplidar import RPLidar
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidarpublisher')

        self.publisher = self.create_publisher(LaserScan, 'lidar', 10) # LaserScan - tip poruke, lidar - ime topic-a, 10 - vel. bafera
        

        self.PORT_NAME = '/dev/ttyUSB0'   # USB port 
        self.MAX_DISTANCE = 10000         # 10m max
        self.lidar = RPLidar(self.PORT_NAME)
        
        # Threading zastita
        self.data_lock = threading.Lock()
        self.latest_scan = []
        self.is_running = True

        # Threadovanje radi, radi kontinualnog citanja
        self.hardware_thread = threading.Thread(target=self.read_lidar)
        self.hardware_thread.daemon = True
        self.hardware_thread.start()

        # Tajmer na 5Hz
        self.timer = self.create_timer(0.2, self.publish_timer_callback)
        
    def read_lidar(self):
        while self.is_running:
            try:
                self.get_logger().info("Povezivanje sa LIDAR-om...")
                self.lidar.clear_input()
                self.lidar.stop_motor()
                time.sleep(0.5) # Dajte vremena hardveru da se smiri
                self.lidar.start_motor()
                time.sleep(0.5)

                # Ovde krece strimovanje podataka
                for scan in self.lidar.iter_scans():
                    if not self.is_running:
                        break
                    with self.data_lock:
                        self.latest_scan = scan
                        
            except Exception as e:
                self.get_logger().warn(f"Greska u strimu ({e}). Pokusavam ponovni reset...")
                try:
                    # Pokusaj potpunog reseta hardvera pre sledeceg pokusaja
                    self.lidar.reset()
                    time.sleep(1.0)
                except Exception:
                    pass # Ako je serijski port zakljucan, probaj sledecu iteraciju petlje
        
    def publish_timer_callback(self):
        with self.data_lock:
            current_data = self.latest_scan
        
        if not current_data:
            return
    
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_frame'

        msg.angle_min = 0.0
        msg.angle_max = math.pi * 2
        msg.angle_increment = msg.angle_max / 360.0
        msg.range_min = 0.15 # 15 cm fiksno
        msg.range_max = 10.0 # 10m promenljivo

        msg.ranges = [float("inf")] * 360

        for (_, angle, distance) in current_data:
            index = int(angle) % 360 # indeks of 0 do 359
            msg.ranges[index] = distance / 1000.0 # Lidar daje mm, ROS2 ocekuje m
        
        self.publisher.publish(msg)

    def stop(self):
        self.is_running = False
        self.lidar.stop()
        self.lidar.disconnect()


def main(args = None):
    rclpy.init(args = args)

    lidarpublisher = LidarPublisher()
    
    try:
        rclpy.spin(lidarpublisher)
    except KeyboardInterrupt:
        print("Kraj!")
    finally:
        lidarpublisher.stop()
        lidarpublisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




### UPUTSTVO

# Step 1: Launch your custom Node

# Run your Python script in one terminal window:
# Bash

# python3 your_lidar_node.py

# Step 2: Launch RViz 2

# Open a brand new terminal window, source your ROS 2 environment, and launch RViz:
# Bash

# rviz2

# A blank 3D grid window will pop up.
# Step 3: Configure RViz to see your LIDAR data

# Because RViz is blank by default, you have to tell it what to look for. Follow these quick steps in the RViz GUI:

#     Fix the Coordinate Frame: On the left panel under Global Options, find Fixed Frame. It will likely say map. Click it and change it manually to text you typed in your code: lidar_frame.

#     Add the LaserScan Display: At the bottom left, click the Add button. A list of display types will appear. Select LaserScan and click OK.

#     Point it to your Topic: On the left panel, you will see a new LaserScan entry. Expand it and find the Topic setting. Click the empty drop-down, and select your topic: /lidar.

# Step 4: Make the dots easier to see (Optional)

# By default, RViz might render your laser returns as tiny, single-pixel dots. To make them highly visible:

#     Inside the LaserScan dropdown on the left panel, find Size (m).

#     Change it from 0.01 to something like 0.05 or 0.1 to make the scan points appear as thick, bright squares.