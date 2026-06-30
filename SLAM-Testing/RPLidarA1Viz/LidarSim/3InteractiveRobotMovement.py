# Empty the scans/interactive folder before usage

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import matplotlib.patches as patches
import matplotlib.colors as mcolors

class InteractiveLiDARSimulator:
    def __init__(self, square_size=10, angular_resolution=0.8):
        """
        Initialize interactive LiDAR simulator
        
        Parameters:
        square_size: size of the square space (default 10 units)
        angular_resolution: angular resolution in degrees (default 0.8)
        """
        self.square_size = square_size
        self.half_size = square_size / 2
        self.angular_resolution = angular_resolution
        self.angles = np.arange(0, 360, angular_resolution)
        
        # Define two circles with different sizes and positions (offset)
        self.circle1_center = np.array([-1.5, 0.5])
        self.circle1_radius = 1.0
        self.circle2_center = np.array([2.0, 1.5])
        self.circle2_radius = 1.8
        
        # Robot state
        self.robot_position = np.array([0.0, 0.0])
        self.robot_angle = 0  # Facing direction in degrees
        self.scans = []  # Store raw scan data: [[distance, angle], ...]
        self.scan_positions = []  # Store positions where scans were taken
        self.trajectory = [[0.0, 0.0]]
        self.move_step = 0.3
        
        # Initialize at valid position
        if not self.is_position_valid(self.robot_position):
            self.robot_position = self.find_valid_position(self.robot_position)
        
        # Setup plot
        self.fig = None
        self.ax_main = None
        self.robot_marker = None
        
    def is_position_valid(self, position):
        """Check if a position is valid (inside square and not inside circles)"""
        x, y = position
        margin = 0.1
        
        # Check if inside square
        if abs(x) > self.half_size - margin or abs(y) > self.half_size - margin:
            return False
        
        # Check if inside circle 1
        dist_to_circle1 = np.linalg.norm(position - self.circle1_center)
        if dist_to_circle1 < self.circle1_radius + margin:
            return False
        
        # Check if inside circle 2
        dist_to_circle2 = np.linalg.norm(position - self.circle2_center)
        if dist_to_circle2 < self.circle2_radius + margin:
            return False
        
        return True
    
    def find_valid_position(self, target_position, max_attempts=100):
        """Find a valid position near the target position"""
        if self.is_position_valid(target_position):
            return target_position
        
        # Try random positions near the target
        for _ in range(max_attempts):
            offset = np.random.uniform(-2, 2, 2)
            candidate = target_position + offset
            candidate = np.clip(candidate, 
                              [-self.half_size + 0.2, -self.half_size + 0.2],
                              [self.half_size - 0.2, self.half_size - 0.2])
            
            if self.is_position_valid(candidate):
                return candidate
        
        # If no valid position found, return a safe position
        safe_positions = [
            np.array([0.0, 0.0]),
            np.array([1.0, 0.0]),
            np.array([-1.0, 0.0]),
            np.array([0.0, 1.0]),
            np.array([0.0, -1.0])
        ]
        
        for pos in safe_positions:
            if self.is_position_valid(pos):
                return pos
        
        raise ValueError("No valid position found in the space!")
    
    def move_robot(self, direction):
        """
        Move robot in specified direction with collision checking
        
        Parameters:
        direction: 'up', 'down', 'left', 'right'
        """
        step = self.move_step
        
        # Calculate new position
        if direction == 'up':
            new_pos = self.robot_position + np.array([0, step])
            self.robot_angle = 90
        elif direction == 'down':
            new_pos = self.robot_position + np.array([0, -step])
            self.robot_angle = -90
        elif direction == 'left':
            new_pos = self.robot_position + np.array([-step, 0])
            self.robot_angle = 180
        elif direction == 'right':
            new_pos = self.robot_position + np.array([step, 0])
            self.robot_angle = 0
        
        # Check if new position is valid
        if self.is_position_valid(new_pos):
            self.robot_position = new_pos
            self.trajectory.append(new_pos.copy())
            return True
        else:
            # Try smaller step
            smaller_step = step * 0.5
            if direction == 'up':
                new_pos = self.robot_position + np.array([0, smaller_step])
            elif direction == 'down':
                new_pos = self.robot_position + np.array([0, -smaller_step])
            elif direction == 'left':
                new_pos = self.robot_position + np.array([-smaller_step, 0])
            elif direction == 'right':
                new_pos = self.robot_position + np.array([smaller_step, 0])
            
            if self.is_position_valid(new_pos):
                self.robot_position = new_pos
                self.trajectory.append(new_pos.copy())
                return True
            
            # Try moving perpendicular if blocked
            if direction in ['up', 'down']:
                # Try left and right
                for side in ['left', 'right']:
                    if side == 'left':
                        side_pos = self.robot_position + np.array([-step*0.3, 0])
                    else:
                        side_pos = self.robot_position + np.array([step*0.3, 0])
                    
                    if self.is_position_valid(side_pos):
                        # Move diagonally
                        if direction == 'up':
                            diag_pos = side_pos + np.array([0, step*0.7])
                        else:
                            diag_pos = side_pos + np.array([0, -step*0.7])
                        
                        if self.is_position_valid(diag_pos):
                            self.robot_position = diag_pos
                            self.trajectory.append(diag_pos.copy())
                            return True
            else:
                # Try up and down
                for side in ['up', 'down']:
                    if side == 'up':
                        side_pos = self.robot_position + np.array([0, step*0.3])
                    else:
                        side_pos = self.robot_position + np.array([0, -step*0.3])
                    
                    if self.is_position_valid(side_pos):
                        if direction == 'left':
                            diag_pos = side_pos + np.array([-step*0.7, 0])
                        else:
                            diag_pos = side_pos + np.array([step*0.7, 0])
                        
                        if self.is_position_valid(diag_pos):
                            self.robot_position = diag_pos
                            self.trajectory.append(diag_pos.copy())
                            return True
            
            print(f"Cannot move {direction}, obstacle in the way!")
            return False
    
    def take_scan(self):
        """Perform a LiDAR scan at current position and store raw data"""
        scan_data = self.scan(self.robot_position)
        
        # Store raw scan data (distance, angle pairs)
        self.scans.append(scan_data)
        self.scan_positions.append(self.robot_position.copy())
        
        print(f"Scan taken at position ({self.robot_position[0]:.2f}, {self.robot_position[1]:.2f})")
        print(f"  Points captured: {len(scan_data)}")
        print(f"  Min distance: {scan_data[:, 0].min():.3f}")
        print(f"  Max distance: {scan_data[:, 0].max():.3f}")
        print(f"  Avg distance: {scan_data[:, 0].mean():.3f}")
        print("-" * 40)
        
        self.update_plot()
        return scan_data
    
    def get_intersection_distance(self, origin, angle):
        """Calculate distance to first intersection with any obstacle"""
        theta = np.radians(angle)
        direction = np.array([np.cos(theta), np.sin(theta)])
        
        # Define the square boundaries
        square_vertices = np.array([
            [-self.half_size, -self.half_size],
            [self.half_size, -self.half_size],
            [self.half_size, self.half_size],
            [-self.half_size, self.half_size],
            [-self.half_size, -self.half_size]
        ])
        
        distances = []
        
        # Check intersection with square edges
        for i in range(4):
            p1 = square_vertices[i]
            p2 = square_vertices[i+1]
            intersection = self.ray_segment_intersection(origin, direction, p1, p2)
            if intersection is not None:
                dist = np.linalg.norm(intersection - origin)
                if dist > 0.001:
                    distances.append(dist)
        
        # Check intersection with circles
        circle1_intersection = self.ray_circle_intersection(origin, direction, 
                                                           self.circle1_center, 
                                                           self.circle1_radius)
        if circle1_intersection is not None:
            distances.append(np.linalg.norm(circle1_intersection - origin))
        
        circle2_intersection = self.ray_circle_intersection(origin, direction, 
                                                           self.circle2_center, 
                                                           self.circle2_radius)
        if circle2_intersection is not None:
            distances.append(np.linalg.norm(circle2_intersection - origin))
        
        if distances:
            min_dist = min(distances)
            noise = np.random.normal(0, 0.02 * min_dist)
            return max(min_dist + noise, 0.01)
        else:
            return None
    
    def ray_segment_intersection(self, origin, direction, p1, p2):
        """Find intersection between ray and line segment"""
        p1 = np.array(p1)
        p2 = np.array(p2)
        d = p2 - p1
        v = direction
        
        det = v[0]*d[1] - v[1]*d[0]
        if abs(det) < 1e-10:
            return None
        
        t = ((p1[0] - origin[0])*d[1] - (p1[1] - origin[1])*d[0]) / det
        u = ((p1[0] - origin[0])*v[1] - (p1[1] - origin[1])*v[0]) / det
        
        if t > 0 and 0 <= u <= 1:
            return origin + t * v
        return None
    
    def ray_circle_intersection(self, origin, direction, center, radius):
        """Find intersection between ray and circle"""
        oc = origin - center
        a = np.dot(direction, direction)
        b = 2 * np.dot(oc, direction)
        c = np.dot(oc, oc) - radius**2
        
        discriminant = b**2 - 4*a*c
        
        if discriminant < 0:
            return None
        
        sqrt_disc = np.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2*a)
        t2 = (-b + sqrt_disc) / (2*a)
        
        if t1 > 0:
            return origin + t1 * direction
        elif t2 > 0:
            return origin + t2 * direction
        return None
    
    def scan(self, origin):
        """Perform a full 360-degree scan from a given origin"""
        scan_data = []
        
        for angle in self.angles:
            distance = self.get_intersection_distance(origin, angle)
            if distance is not None:
                scan_data.append([distance, angle])
        
        return np.array(scan_data)
    
    def setup_plot(self):
        """Setup the interactive plot"""
        self.fig = plt.figure(figsize=(12, 10))
        self.fig.suptitle('Interactive LiDAR Simulator - WASD to Move, Enter to Scan', 
                         fontsize=14, fontweight='bold')
        
        # Main plot
        self.ax_main = self.fig.add_subplot(111)
        self.setup_main_plot()
        
        # Connect keyboard events
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Add instruction text
        self.add_instructions()
        
        plt.tight_layout()
        return self.fig
    
    def setup_main_plot(self):
        """Setup the main environment plot"""
        ax = self.ax_main
        ax.clear()
        
        half_size = self.half_size
        
        # Plot square
        square = Rectangle([-half_size, -half_size], 
                          self.square_size, self.square_size,
                          linewidth=2, edgecolor='black', facecolor='none')
        ax.add_patch(square)
        
        # Plot circle 1
        circle1 = Circle(self.circle1_center, self.circle1_radius,
                        linewidth=2, edgecolor='red', facecolor='red', alpha=0.3)
        ax.add_patch(circle1)
        
        # Plot circle 2
        circle2 = Circle(self.circle2_center, self.circle2_radius,
                        linewidth=2, edgecolor='blue', facecolor='blue', alpha=0.3)
        ax.add_patch(circle2)
        
        # Plot trajectory
        if len(self.trajectory) > 1:
            traj = np.array(self.trajectory)
            ax.plot(traj[:, 0], traj[:, 1], 'g--', alpha=0.5, linewidth=1, label='Trajectory')
        
        # Plot start position
        if len(self.trajectory) > 0:
            ax.scatter(self.trajectory[0][0], self.trajectory[0][1], 
                      c='green', s=80, marker='*', edgecolor='black', 
                      linewidth=2, label='Start', zorder=5)
        
        # Plot robot
        robot_pos = self.robot_position
        self.robot_marker = ax.scatter(robot_pos[0], robot_pos[1], 
                                       c='red', s=150, marker='o', 
                                       edgecolor='black', linewidth=2, 
                                       label='Robot', zorder=10)
        
        # Add robot direction indicator
        angle_rad = np.radians(self.robot_angle)
        arrow_length = 0.5
        arrow_end = robot_pos + np.array([arrow_length * np.cos(angle_rad), 
                                          arrow_length * np.sin(angle_rad)])
        ax.annotate('', xy=arrow_end, xytext=robot_pos,
                   arrowprops=dict(arrowstyle='->', lw=2, color='darkred'))
        
        # Plot existing scans
        if self.scans:
            colors = plt.cm.viridis(np.linspace(0, 1, len(self.scans)))
            for i, scan_data in enumerate(self.scans):
                pos = self.scan_positions[i]
                if len(scan_data) > 0:
                    x = scan_data[:, 0] * np.cos(np.radians(scan_data[:, 1])) + pos[0]
                    y = scan_data[:, 0] * np.sin(np.radians(scan_data[:, 1])) + pos[1]
                    ax.scatter(x, y, c=[colors[i]], s=3, alpha=0.5)
                    # Mark scan position
                    ax.scatter(pos[0], pos[1], c=[colors[i]], s=40, 
                              marker='o', edgecolor='black', linewidth=0.5)
        
        # Set axis properties
        ax.set_xlim(-half_size-1, half_size+1)
        ax.set_ylim(-half_size-1, half_size+1)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_title('Environment with LiDAR Scans')
        ax.legend(loc='upper right')
    
    def add_instructions(self):
        """Add instruction text to the plot"""
        instr_text = """
        Controls:
        • W - Move Up
        • A - Move Left  
        • X - Move Down
        • D - Move Right
        • ENTER - Take Scan
        • ESC - Close/Quit
        """
        self.fig.text(0.02, 0.02, instr_text, fontsize=10, 
                     bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.8))
    
    def update_plot(self):
        """Update the plot with current state"""
        self.setup_main_plot()
        self.fig.canvas.draw()
    
    def on_key_press(self, event):
        """Handle key press events"""
        if event.key == 'w':
            self.move_robot('up')
            self.update_plot()
        elif event.key == 'a':
            self.move_robot('left')
            self.update_plot()
        elif event.key == 'x':
            self.move_robot('down')
            self.update_plot()
        elif event.key == 'd':
            self.move_robot('right')
            self.update_plot()
        elif event.key == 'enter':
            self.take_scan()
        elif event.key == 'escape':
            plt.close()
            print("\nClosing application...")
    
    def get_raw_scans(self):
        """Return all raw scan data as a list of numpy arrays"""
        return self.scans
    
    def get_scan_positions(self):
        """Return positions where scans were taken"""
        return self.scan_positions
    
    def get_scan_data_as_list(self):
        """Return all scan data as a single list of (position, scan_data) tuples"""
        return list(zip(self.scan_positions, self.scans))
    
    def save_scans(self, prefix="lidar_scan"):
        """Save all scans to CSV files"""
        if not self.scans:
            print("No scans to save!")
            return
        
        print(f"\nSaving {len(self.scans)} scans...")
        for i, (position, scan_data) in enumerate(zip(self.scan_positions, self.scans)):
            filename = f"{prefix}_{i+1:03d}.csv"
            np.savetxt('scans/interactive/'+filename, scan_data, delimiter=',', 
                      #header=f'Position: ({position[0]:.3f}, {position[1]:.3f}), distance,angle', 
                      comments='')
            print(f"  Saved scan {i+1} to {filename}")
        
        # Also save scan metadata
        #metadata_filename = f"{prefix}_metadata.csv"
        #metadata = np.array([[pos[0], pos[1], len(scan)] 
        #                    for pos, scan in zip(self.scan_positions, self.scans)])
        #np.savetxt(metadata_filename, metadata, delimiter=',',
        #          #header='x_position,y_position,num_points', comments='')
        #print(f"  Saved metadata to {metadata_filename}")
        print("-" * 40)
    
    def run(self):
        """Run the interactive simulation"""
        self.setup_plot()
        print("=" * 60)
        print("Interactive LiDAR Simulator")
        print("=" * 60)
        print("Controls:")
        print("  W - Move Up")
        print("  A - Move Left")
        print("  X - Move Down")
        print("  D - Move Right")
        print("  ENTER - Take a LiDAR scan")
        print("  ESC - Close the application")
        print("=" * 60)
        print(f"Starting position: ({self.robot_position[0]:.2f}, {self.robot_position[1]:.2f})")
        print("Start moving and scanning!")
        print("=" * 60)
        plt.show()
        
        # Save data when closed
        self.save_scans()
        
        # Print summary
        print("\n" + "=" * 60)
        print("Summary:")
        print(f"  Total scans taken: {len(self.scans)}")
        print(f"  Trajectory points: {len(self.trajectory)}")
        if self.scans:
            total_points = sum(len(scan) for scan in self.scans)
            print(f"  Total LiDAR points: {total_points}")
        print(f"  Final position: ({self.robot_position[0]:.2f}, {self.robot_position[1]:.2f})")
        print("=" * 60)

def main():
    """Main execution function"""
    # Create interactive simulator
    simulator = InteractiveLiDARSimulator(square_size=10, angular_resolution=0.8)
    
    # Run the interactive simulation
    simulator.run()
    
    return simulator

if __name__ == "__main__":
    simulator = main()