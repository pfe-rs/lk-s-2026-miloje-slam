import serial
import time

class MotionController:
    def __init__(self, port='/dev/ttyACM0', baudrate=9600):
        # We set a small timeout so reading doesn't block the SLAM loop
        self.ser = serial.Serial(port, baudrate, timeout=0.01)
        time.sleep(2)  # Wait for Arduino to reset after connection
        print("Motion Controller Initialized.")

    def send_move_command(self, motor1, direction, speed, duration):
        """
        Sends command formatted exactly as: "3 F 400 500\n"
        Assumes motor1 is the first motor (e.g., 3) and motor2 is next (e.g., 4)
        """
        command = f"3 {direction} {speed} {duration}\n"
        self.ser.write(command.encode('utf-8'))
        print(f"Sent: {command.strip()}")

    def check_odometry(self):
        """
        Non-blocking read to check if Arduino has sent back odometry data.
        Returns data if available, else None.
        """
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith("ODOM:"):
                    # Example expected format: "ODOM:x_ticks,y_ticks"
                    data = line.replace("ODOM:", "").split(",")
                    left_ticks = int(data[0])
                    right_ticks = int(data[1])
                    return left_ticks, right_ticks
            except (ValueError, IndexError, UnicodeDecodeError) as e:
                # Handle corrupted serial frames gracefully
                pass
        return None

# Example usage inside your main SLAM loop
if __name__ == "__main__":
    controller = MotionController()

    # Simulate your SLAM / path_planner main loop
    while True:
        # 1. Update SLAM and path planning calculations...

        # 2. Example: Trigger a forward movement command
        # controller.send_move_command(3, 'F', 400, 500)

        # 3. Constantly check for incoming odometry data without blocking
        odom_data = controller.check_odometry()
        if odom_data:
            left, right = odom_data
            print(f"Updated SLAM Odometry: Left={left}, Right={right}")

        time.sleep(0.1) # Small loop sleep
