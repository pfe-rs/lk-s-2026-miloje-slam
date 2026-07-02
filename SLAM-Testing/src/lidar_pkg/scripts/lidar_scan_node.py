#!/usr/bin/env python3
"""
- Uzima jedan sken na pocetku
- Ne radi nista dok neki drugi node ne pozove '~/scan_request' servis (std_srvs/Trigger)
- Na trigeru uzima 360 stepeni sken i salje ga u sensor_msgs/LaserScan i salje na ~/scan
- Stand by posle

Blokator lidar I/O se izvrsava u pozadinskom tredu pa ROS nije blokiran dok ceka hardver
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from lidar_msgs.msg import LidarSweep
from std_srvs.srv import Trigger

from rplidar import RPLidar


class LidarScanNode(Node):

    def __init__(self):
        super().__init__('lidar_scan_node')

        # ---- Parametri ----
        self.declare_parameter('port_name', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('motor_stabilize_sec', 2.0)

        self._port_name = self.get_parameter('port_name').value
        self._frame_id = self.get_parameter('frame_id').value
        self._stabilize_sec = self.get_parameter('motor_stabilize_sec').value

        # ---- Lidar veza ----
        self._lidar = None
        self._lidar_lock = threading.Lock()  # cuvar pristupa
        self._connect_lidar()

        # ---- Pablisher za sken ----
        self._scan_pub = self.create_publisher(LidarSweep, '~/scan', 10)

        # ---- Cekanje servisa ----
        self._scan_cb_group = ReentrantCallbackGroup()
        self._scan_service = self.create_service(
            Trigger,
            '~/scan_request',
            self._handle_scan_request,
            callback_group=self._scan_cb_group,
        )

        # Ne dozvoljava dva preklapajuca skena
        self._scan_in_progress = threading.Lock()

        self.get_logger().info(
            f"Lidar ready on {self._port_name}. "
            f"Waiting for calls to 'scan_request' service."
        )

    # Setup

    def _connect_lidar(self):
        self._lidar = RPLidar(self._port_name)
        self.get_logger().info(f'RPLidar Info: {self._lidar.get_info()}')
        self.get_logger().info(f'Health status: {self._lidar.get_health()}')
        self.get_logger().info('Cekanje motora...')
        time.sleep(self._stabilize_sec)

    def destroy_node(self):
        if self._lidar is not None:
            try:
                self._lidar.stop()
                self._lidar.stop_motor()
                self._lidar.disconnect()
                self.get_logger().info('Lidar uspesno otkacen.')
            except Exception as e:
                self.get_logger().warn(f'Greska pri otkacenju: {e}')
        super().destroy_node()

    # Servis callback

    def _handle_scan_request(self, request, response):
        """ Zove ga triger, blokira dok se jedna rotacija sakuplja i salje"""

        if not self._scan_in_progress.acquire(blocking=False):
            response.success = False
            response.message = 'Sken se izvrsava'
            return response

        try:
            points = self._collect_one_rotation()
        except Exception as e:
            self.get_logger().error(f'Sken neuspesan: {e}')
            response.success = False
            response.message = f'Sken neuspesan: {e}'
            return response
        finally:
            self._scan_in_progress.release()

        if not points:
            response.success = False
            response.message = 'Nisu pokupljene validne tacke'
            return response

        self._publish_scan(points)
        response.success = True
        response.message = f'Sken zavrsen: {len(points)} tacaka objavljeno.'
        return response

    # Logika prikupljanja skena
    
    def _collect_one_rotation(self):
        """Uzima jedan sken od 360 stepeni, 
        u formatu (ugao, udaljenost) [stepen, mm] sortiran po uglu"""
        
        raw_points = []
        start_collecting = False

        self.get_logger().info('Ceka pocetak skena')

        with self._lidar_lock:
            for new_scan, _, angle, distance in self._lidar.iter_measures():
                if new_scan:
                    if not start_collecting:
                        start_collecting = True
                        self.get_logger().info(
                            'Sken zapoceo, skenira 360'
                        )
                        if distance > 0:
                            raw_points.append((angle, distance))
                        continue
                    else:
                        self.get_logger().info(
                            'Sken gotov, zavrsava'
                        )
                        break

                if start_collecting and distance > 0:
                    raw_points.append((angle, distance))

        raw_points.sort(key=lambda p: p[0])
        return raw_points

    # Pablish

    def _publish_scan(self, points):
        # Objavlju tacke kao LidarSweep poruke (custom format)
        
        msg = LidarSweep()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.angles = [float(angle) for angle, _ in points]
        msg.distances = [float(distance) for _, distance in points]

        self._scan_pub.publish(msg)
        self.get_logger().info(f'Objavljen LidarSweep sa {len(points)} tacaka.')


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
