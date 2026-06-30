import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import matplotlib.colors as mcolors

class LiDARSimulator:
    def __init__(self, square_size=10, angular_resolution=0.8):
        """
        Initialize LiDAR simulator
        
        Parameters:
        square_size: size of the square space (default 10 units)
        angular_resolution: angular resolution in degrees (default 0.8)
        """
        self.square_size = square_size
        self.half_size = square_size / 2
        self.angular_resolution = angular_resolution
        self.angles = np.arange(0, 360, angular_resolution)
        
        # Define two circles with different sizes and positions (offset)
        # Circle 1: smaller, positioned left
        self.circle1_center = np.array([-1.5, 0.5])
        self.circle1_radius = 1.0
        
        # Circle 2: larger, positioned right and slightly up
        self.circle2_center = np.array([2.0, 1.5])
        self.circle2_radius = 1.8
    
    def is_position_valid(self, position):
        """
        Check if a position is valid (inside square and not inside circles)
        
        Parameters:
        position: [x, y] position to check
        
        Returns:
        bool: True if valid, False otherwise
        """
        x, y = position
        
        # Check if inside square (with small margin)
        margin = 0.1  # Small buffer from walls
        if abs(x) > self.half_size - margin or abs(y) > self.half_size - margin:
            return False
        
        # Check if inside circle 1 (with small margin)
        dist_to_circle1 = np.linalg.norm(position - self.circle1_center)
        if dist_to_circle1 < self.circle1_radius + margin:
            return False
        
        # Check if inside circle 2 (with small margin)
        dist_to_circle2 = np.linalg.norm(position - self.circle2_center)
        if dist_to_circle2 < self.circle2_radius + margin:
            return False
        
        return True
    
    def find_valid_position(self, target_position, max_attempts=100):
        """
        Find a valid position near the target position
        
        Parameters:
        target_position: desired [x, y] position
        max_attempts: maximum number of random attempts
        
        Returns:
        numpy array: valid position
        """
        if self.is_position_valid(target_position):
            return target_position
        
        # Try random positions near the target
        for _ in range(max_attempts):
            # Random offset within 2 units
            offset = np.random.uniform(-2, 2, 2)
            candidate = target_position + offset
            
            # Keep within square bounds
            candidate = np.clip(candidate, 
                              [-self.half_size + 0.2, -self.half_size + 0.2],
                              [self.half_size - 0.2, self.half_size - 0.2])
            
            if self.is_position_valid(candidate):
                return candidate
        
        # If no valid position found, try grid search
        print("Warning: Random search failed, using grid search...")
        for x in np.linspace(-self.half_size + 0.5, self.half_size - 0.5, 20):
            for y in np.linspace(-self.half_size + 0.5, self.half_size - 0.5, 20):
                candidate = np.array([x, y])
                if self.is_position_valid(candidate):
                    return candidate
        
        raise ValueError("No valid position found in the space!")
    
    def move_robot(self, current_position, target_position, step_size=0.5):
        """
        Move robot from current position toward target position with collision checking
        
        Parameters:
        current_position: current [x, y] position
        target_position: desired [x, y] position
        step_size: maximum step size per move
        
        Returns:
        numpy array: new valid position
        """
        # Calculate direction vector
        direction = target_position - current_position
        distance = np.linalg.norm(direction)
        
        if distance < 0.01:
            return current_position
        
        # Normalize direction
        direction = direction / distance
        
        # Try to move step_size distance in the direction
        candidate = current_position + direction * min(step_size, distance)
        
        # Check if candidate is valid
        if self.is_position_valid(candidate):
            return candidate
        
        # If not valid, try smaller steps
        for fraction in [0.75, 0.5, 0.25, 0.1]:
            candidate = current_position + direction * min(step_size * fraction, distance)
            if self.is_position_valid(candidate):
                return candidate
        
        # If still not valid, try moving in perpendicular directions
        perp_direction = np.array([-direction[1], direction[0]])
        for sign in [1, -1]:
            for fraction in [0.5, 0.3, 0.1]:
                candidate = current_position + (direction * 0.3 + sign * perp_direction * 0.7) * min(step_size * fraction, distance)
                if self.is_position_valid(candidate):
                    return candidate
        
        # If nothing works, stay in place
        print(f"Warning: Cannot move from {current_position} to {target_position}")
        return current_position
    
    def get_intersection_distance(self, origin, angle):
        """
        Calculate distance to first intersection with any obstacle
        """
        # Convert angle to radians
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
        
        # Find intersection with square
        distances = []
        
        # Check intersection with square edges
        for i in range(4):
            p1 = square_vertices[i]
            p2 = square_vertices[i+1]
            
            # Line segment intersection
            intersection = self.ray_segment_intersection(origin, direction, p1, p2)
            if intersection is not None:
                dist = np.linalg.norm(intersection - origin)
                if dist > 0.001:  # avoid self-intersection
                    distances.append(dist)
        
        # Check intersection with circle 1
        circle1_intersection = self.ray_circle_intersection(origin, direction, 
                                                           self.circle1_center, 
                                                           self.circle1_radius)
        if circle1_intersection is not None:
            distances.append(np.linalg.norm(circle1_intersection - origin))
        
        # Check intersection with circle 2
        circle2_intersection = self.ray_circle_intersection(origin, direction, 
                                                           self.circle2_center, 
                                                           self.circle2_radius)
        if circle2_intersection is not None:
            distances.append(np.linalg.norm(circle2_intersection - origin))
        
        # Return the minimum distance (closest obstacle)
        if distances:
            # Add a small amount of noise to simulate real LiDAR
            min_dist = min(distances)
            # Add Gaussian noise (2% standard deviation)
            noise = np.random.normal(0, 0.02 * min_dist)
            return max(min_dist + noise, 0.01)
        else:
            # No intersection found (shouldn't happen in a closed space)
            return None
    
    def ray_segment_intersection(self, origin, direction, p1, p2):
        """Find intersection between ray and line segment"""
        p1 = np.array(p1)
        p2 = np.array(p2)
        d = p2 - p1
        v = direction
        
        # Solve for parameters: origin + t*v = p1 + u*d
        det = v[0]*d[1] - v[1]*d[0]
        if abs(det) < 1e-10:
            return None  # Ray and line are parallel
        
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
        
        # Find closest positive intersection
        sqrt_disc = np.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2*a)
        t2 = (-b + sqrt_disc) / (2*a)
        
        if t1 > 0:
            return origin + t1 * direction
        elif t2 > 0:
            return origin + t2 * direction
        return None
    
    def scan(self, origin, add_noise=True):
        """
        Perform a full 360-degree scan from a given origin
        
        Returns:
        numpy array of (distance, angle) pairs
        """
        scan_data = []
        
        for angle in self.angles:
            distance = self.get_intersection_distance(origin, angle)
            if distance is not None:
                scan_data.append([distance, angle])
        
        return np.array(scan_data)

class RobotNavigator:
    def __init__(self, simulator):
        """
        Initialize robot navigator with collision checking
        """
        self.simulator = simulator
        self.current_position = None
        self.trajectory = []
        self.scans = []
        
        # Start at a valid position
        start_pos = np.array([0.0, 0.0])
        if not simulator.is_position_valid(start_pos):
            start_pos = simulator.find_valid_position(start_pos)
        self.current_position = start_pos
        self.trajectory.append(start_pos.copy())
    
    def move_to(self, target_position, step_size=0.5, scan_after_move=True):
        """
        Move robot to target position with collision checking
        """
        print(f"Moving from {self.current_position} to {target_position}")
        
        # Find valid target position
        valid_target = self.simulator.find_valid_position(target_position)
        print(f"Valid target position: {valid_target}")
        
        # Move step by step
        max_steps = 100
        for step in range(max_steps):
            # Calculate next position
            next_pos = self.simulator.move_robot(self.current_position, valid_target, step_size)
            
            # Check if we've reached the target
            if np.linalg.norm(next_pos - valid_target) < 0.01:
                self.current_position = next_pos
                self.trajectory.append(next_pos.copy())
                if scan_after_move:
                    self._scan_and_store()
                break
            
            # Update position
            self.current_position = next_pos
            self.trajectory.append(next_pos.copy())
            
            # Scan at intermediate positions if desired
            if scan_after_move and step % 2 == 0:
                self._scan_and_store()
        
        return self.current_position
    
    def _scan_and_store(self):
        """Perform a scan at current position and store it"""
        scan_data = self.simulator.scan(self.current_position)
        self.scans.append({
            'position': self.current_position.copy(),
            'data': scan_data
        })
        print(f"Scan at position ({self.current_position[0]:.2f}, {self.current_position[1]:.2f}): {len(scan_data)} points")
    
    def get_scans(self):
        """Return all scans taken during navigation"""
        return self.scans
    
    def get_trajectory(self):
        """Return the robot trajectory"""
        return np.array(self.trajectory)

def visualize_navigation(simulator, robot_navigator, show_scans=True, animate=True):
    """
    Visualize robot navigation with scans
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
    fig.suptitle('Robot Navigation with LiDAR Scans', fontsize=14, fontweight='bold')
    
    # Create the environment
    half_size = simulator.half_size
    
    # Plot square
    square = Rectangle([-half_size, -half_size], 
                      simulator.square_size, simulator.square_size,
                      linewidth=2, edgecolor='black', facecolor='none')
    ax1.add_patch(square)
    
    # Plot circle 1
    circle1 = Circle(simulator.circle1_center, simulator.circle1_radius,
                    linewidth=2, edgecolor='red', facecolor='red', alpha=0.3)
    ax1.add_patch(circle1)
    
    # Plot circle 2
    circle2 = Circle(simulator.circle2_center, simulator.circle2_radius,
                    linewidth=2, edgecolor='blue', facecolor='blue', alpha=0.3)
    ax1.add_patch(circle2)
    
    # Plot trajectory
    trajectory = robot_navigator.get_trajectory()
    if len(trajectory) > 0:
        ax1.plot(trajectory[:, 0], trajectory[:, 1], 'g--', alpha=0.5, label='Trajectory')
        ax1.scatter(trajectory[0, 0], trajectory[0, 1], c='green', s=100, 
                   marker='*', edgecolor='black', linewidth=2, label='Start')
        ax1.scatter(trajectory[-1, 0], trajectory[-1, 1], c='purple', s=100, 
                   marker='*', edgecolor='black', linewidth=2, label='End')
    
    # Plot scans
    scans = robot_navigator.get_scans()
    if show_scans and scans:
        # Create colormap for different scan positions
        colors = plt.cm.viridis(np.linspace(0, 1, len(scans)))
        
        for i, scan_data in enumerate(scans):
            pos = scan_data['position']
            data = scan_data['data']
            
            if len(data) > 0:
                # Convert to Cartesian
                x = data[:, 0] * np.cos(np.radians(data[:, 1])) + pos[0]
                y = data[:, 0] * np.sin(np.radians(data[:, 1])) + pos[1]
                
                # Plot LiDAR points with different colors for each scan
                ax1.scatter(x, y, c=[colors[i]], s=2, alpha=0.5)
                
                # Plot sensor position
                ax1.scatter(pos[0], pos[1], c=[colors[i]], s=50, 
                           marker='o', edgecolor='black', linewidth=1)
    
    ax1.set_xlim(-half_size-1, half_size+1)
    ax1.set_ylim(-half_size-1, half_size+1)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.set_title('Navigation with LiDAR Scans')
    ax1.legend()
    
    # Plot polar scans
    if scans:
        # Plot the latest scan in polar coordinates
        latest_scan = scans[-1]
        data = latest_scan['data']
        pos = latest_scan['position']
        
        if len(data) > 0:
            ax2.scatter(data[:, 1], data[:, 0], c=data[:, 0], 
                       s=10, cmap='viridis', alpha=0.7)
            ax2.set_xlabel('Angle (degrees)')
            ax2.set_ylabel('Distance')
            ax2.set_title(f'Latest Scan at ({pos[0]:.2f}, {pos[1]:.2f})')
            ax2.grid(True, alpha=0.3)
            ax2.set_ylim(0, 8)
            ax2.set_xlim(0, 360)
    
    plt.tight_layout()
    return fig

def main():
    """Main execution function"""
    print("LiDAR Simulator with Robot Navigation")
    print("=" * 50)
    
    # Create simulator
    simulator = LiDARSimulator(square_size=10, angular_resolution=0.8)
    
    # Create robot navigator
    robot = RobotNavigator(simulator)
    
    # Define waypoints for the robot to visit
    waypoints = [
        np.array([0.0, 0.0]),           # Center
        np.array([2.0, -2.0]),          # Bottom right
        np.array([-3.0, -1.0]),         # Left
        np.array([-1.0, 2.5]),          # Top left
        np.array([3.0, 1.0]),           # Top right
    ]
    
    print(f"Starting at: {robot.current_position}")
    print("Moving through waypoints...")
    print("-" * 50)
    
    # Navigate through waypoints
    for i, waypoint in enumerate(waypoints):
        print(f"\nWaypoint {i+1}: {waypoint}")
        robot.move_to(waypoint, step_size=0.5, scan_after_move=True)
        
        # Optional: pause between moves for visualization
        # input("Press Enter to continue...")
    
    print("\n" + "=" * 50)
    print(f"Navigation complete! Total scans: {len(robot.get_scans())}")
    
    # Visualize results
    fig = visualize_navigation(simulator, robot, show_scans=True)
    
    # Save data
    scans = robot.get_scans()
    for i, scan_data in enumerate(scans):
        filename = f'navigation_scan_{i+1:03d}.csv'
        np.savetxt("scans/"+filename, scan_data['data'], 
                  delimiter=',', header='distance,angle', comments='')
        print(f"Saved scan {i+1} to {filename}")
    
    # Save trajectory
    trajectory = robot.get_trajectory()
    np.savetxt('scans/robot_trajectory.csv', trajectory, 
              delimiter=',', header='x,y', comments='')
    print("Saved trajectory to robot_trajectory.csv")
    
    plt.show()
    
    return simulator, robot

if __name__ == "__main__":
    simulator, robot = main()