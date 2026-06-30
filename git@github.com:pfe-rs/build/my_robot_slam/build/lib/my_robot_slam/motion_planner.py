# CTRL+F ### za nedovrsene delove
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import serial
import time
import math

class MotionController(Node):
    def __init__(self):
        super().__init__('motion_planner')
        
        # OGRANICENJA MILOJA
        self.STEPS_PER_METER = 795.77  # 8cm tocak i 200 steps/okretu
        self.DEFAULT_SPEED = 200       # Br. okreta po sekudni
        
        # Serijska veza
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)
            time.sleep(2)  # Ceka Arduino
            self.get_logger().info("Connected to Miloje via Serial.")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")

        # Pretplata na globalni put
        self.path_sub = self.create_subscription(
            Path, 'global_path', self.path_callback, 10
        )
 
        ### DODATI PREPTLATU NA /path_planner KOJI CITA MSG

        # Prati robota
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0  # RADIJANI

    def path_callback(self, msg):
        if not msg.poses:
            return

        self.get_logger().info(f"Executing new path trajectory ({len(msg.poses)} waypoints)...")

        for pose_stamped in msg.poses:
            target_x = pose_stamped.pose.position.x
            target_y = pose_stamped.pose.position.y

            # 1. VEKTOR TRANSLACIJE
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < 0.05:  # DOSAO DO CILJA, GRESKA DO 5cm
                continue

            # NADJI UGAO DO ODREDISTA
            target_theta = math.atan2(dy, dx)
            
            # NAJMANJI MOGUCI OKRET
            delta_theta = target_theta - self.current_theta
            
            # NORMALIZACIJA UGLA
            delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))
            delta_degrees = int(math.degrees(delta_theta))

            # OKRET
            if abs(delta_degrees) > 3:
                self.get_logger().info(f"Turning: {delta_degrees}°")
                ### Protokol: M A 12 [stepeni] PROTOKOL ROTACIJE TREBA NAPRAVITI!!!!!!!!!!!!!!!
                turn_command = f"M A 12 {delta_degrees}\n"
                self.send_to_arduino(turn_command)
                
                # PROCENA VREMENA ZA OKRET
                time.sleep(1.5) 
                self.current_theta = target_theta

            # 4. TRANSLACIJA, IDI NAPRED
            total_steps = int(distance * self.STEPS_PER_METER)
            self.get_logger().info(f"Driving forward: {distance:.2f}m ({total_steps} steps)")
            
            ### Protokol: M A 10 [koraci] [brzina] DEFINISATI PROTOKOL!!!!!!!!!!!
            drive_command = f"M A 10 {total_steps} {self.DEFAULT_SPEED}\n"
            self.send_to_arduino(drive_command)

            # Izracunaj koliko do kraja kretanja
            duration_sec = total_steps / self.DEFAULT_SPEED
            time.sleep(duration_sec + 0.5)  # 0.5 ZA EVENTUALNE GREKSE RACUNA

            # APDEJT POZICIJE NA KRAJU
            self.current_x = target_x
            self.current_y = target_y

    def send_to_arduino(self, command_string):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.write(command_string.encode('utf-8'))
            self.get_logger().info(f"Dispatched: {command_string.strip()}")
        else:
            self.get_logger().warn(f"Mock Output (No Serial): {command_string.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()