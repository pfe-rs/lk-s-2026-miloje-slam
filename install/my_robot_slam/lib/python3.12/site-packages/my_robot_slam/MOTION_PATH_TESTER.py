# AI GENERISAN KOD ZA TESTIRANJE RADA MOTION I PATH PLANERA
import rclpy
from nav_msgs.msg import OccupancyGrid
rclpy.init()
node = rclpy.create_node('mock_map_publisher')
pub = node.create_publisher(OccupancyGrid, 'map', 10)
msg = OccupancyGrid()
msg.header.frame_id = 'map'
msg.info.resolution = 0.1
msg.info.width = 15
msg.info.height = 15
msg.info.origin.position.x = -0.75
msg.info.origin.position.y = -0.75
msg.data = [0]*224 + [-1]  # 224 clear spaces, 1 frontier pixel at the end
import time
time.sleep(1)
pub.publish(msg)
node.get_logger().info('Test Map Dispatched!')
time.sleep(0.5)
