import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import matplotlib.patches as patches

class LiDARSimulator:
    def __init__(self, square_size=10, angular_resolution=0.8):
        """
        Initialize LiDAR simulator
        
        Parameters:
        square_size: size of the square space (default 10 units)
        angular_resolution: angular resolution in degrees (default 0.8)
        """
        self.square_size = square_size
        self.angular_resolution = angular_resolution
        self.angles = np.arange(0, 360, angular_resolution)
        
        # Define two circles with different sizes and positions (offset)
        # Circle 1: smaller, positioned left
        self.circle1_center = np.array([-1.5, 0.5])
        self.circle1_radius = 1.0
        
        # Circle 2: larger, positioned right and slightly up
        self.circle2_center = np.array([2.0, 1.5])
        self.circle2_radius = 1.8
    
    def get_intersection_distance(self, origin, angle):
        """
        Calculate distance to first intersection with any obstacle
        """
        # Convert angle to radians
        theta = np.radians(angle)
        direction = np.array([np.cos(theta), np.sin(theta)])
        
        # Define the square boundaries
        half_size = self.square_size / 2
        square_vertices = np.array([
            [-half_size, -half_size],
            [half_size, -half_size],
            [half_size, half_size],
            [-half_size, half_size],
            [-half_size, -half_size]
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

def generate_three_scans():
    """Generate three distinct stationary measurements"""
    np.random.seed(42)  # For reproducibility
    
    simulator = LiDARSimulator(square_size=10, angular_resolution=0.8)
    
    # Three different sensor positions
    positions = [
        np.array([0.0, 0.0]),      # Center
        np.array([2.0, -2.0]),      # Right of center
        np.array([-3.0, -1.0])     # Left of center
    ]
    
    scans = []
    positions_used = []
    
    for i, pos in enumerate(positions):
        scan_data = simulator.scan(pos)
        scans.append(scan_data)
        positions_used.append(pos)
        print(f"Scan {i+1} from position ({pos[0]:.1f}, {pos[1]:.1f}): {len(scan_data)} points")
    
    return scans, positions_used, simulator

def visualize_scans(scans, positions, simulator):
    """Visualize the scan data"""
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle('LiDAR Scans of Square Space with Two Offset Circles', fontsize=14, fontweight='bold')
    
    # Create the environment
    half_size = simulator.square_size / 2
    
    for idx, (ax, scan_data, origin) in enumerate(zip(axes, scans, positions)):
        # Plot square
        square = Rectangle([-half_size, -half_size], 
                          simulator.square_size, simulator.square_size,
                          linewidth=2, edgecolor='black', facecolor='none')
        ax.add_patch(square)
        
        # Plot circle 1
        circle1 = Circle(simulator.circle1_center, simulator.circle1_radius,
                        linewidth=2, edgecolor='red', facecolor='red', alpha=0.3)
        ax.add_patch(circle1)
        
        # Plot circle 2
        circle2 = Circle(simulator.circle2_center, simulator.circle2_radius,
                        linewidth=2, edgecolor='blue', facecolor='blue', alpha=0.3)
        ax.add_patch(circle2)
        
        # Plot scan points
        if len(scan_data) > 0:
            # Convert polar to Cartesian
            x = scan_data[:, 0] * np.cos(np.radians(scan_data[:, 1]))
            y = scan_data[:, 0] * np.sin(np.radians(scan_data[:, 1]))
            
            # Add origin offset
            x += origin[0]
            y += origin[1]
            
            # Plot LiDAR points
            ax.scatter(x, y, c=scan_data[:, 0], s=3, cmap='viridis', 
                      alpha=0.7, vmin=0, vmax=10)
            
            # Plot origin (sensor position)
            ax.scatter(origin[0], origin[1], c='red', s=100, marker='*', 
                      edgecolor='black', linewidth=2, label='Sensor', zorder=10)
        
        # Plot square corners for reference
        corners = [[-half_size, -half_size], [half_size, -half_size], 
                   [half_size, half_size], [-half_size, half_size]]
        for corner in corners:
            ax.scatter(corner[0], corner[1], c='black', s=20, marker='+')
        
        # Set axis properties
        ax.set_xlim(-half_size-1, half_size+1)
        ax.set_ylim(-half_size-1, half_size+1)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_title(f'Scan {idx+1}: Origin ({origin[0]:.1f}, {origin[1]:.1f})')
        
        # Add colorbar
        if len(scan_data) > 0:
            cbar = plt.colorbar(ax.collections[0], ax=ax, shrink=0.8)
            cbar.set_label('Distance')
    
    plt.tight_layout()
    return fig

def visualize_polar_scans(scans, positions):
    """Visualize scans in polar coordinates (distance vs angle)"""
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    fig.suptitle('LiDAR Scans in Polar Coordinates', fontsize=14, fontweight='bold')
    
    for idx, (ax, scan_data, pos) in enumerate(zip(axes, scans, positions)):
        if len(scan_data) > 0:
            ax.scatter(scan_data[:, 1], scan_data[:, 0], c=scan_data[:, 0], 
                      s=5, cmap='viridis', alpha=0.7)
            ax.set_xlabel('Angle (degrees)')
            ax.set_ylabel('Distance')
            ax.set_title(f'Scan {idx+1}: Origin ({pos[0]:.1f}, {pos[1]:.1f})')
            ax.grid(True, alpha=0.3)
            ax.set_ylim(0, 8)
            ax.set_xlim(0, 360)
    
    plt.tight_layout()
    return fig

def main():
    """Main execution function"""
    print("Generating LiDAR data for square with two offset circles...")
    print("Angular resolution: 0.8 degrees")
    print("Number of measurements: 3 stationary scans")
    print("-" * 50)
    
    # Generate three scans
    scans, positions, simulator = generate_three_scans()
    
    # Print first few data points
    print("\nSample data points (first 5 from Scan 1):")
    print("Format: [Distance, Angle]")
    for i in range(min(5, len(scans[0]))):
        print(f"  {scans[0][i]}")
    
    # Visualize in Cartesian coordinates
    fig1 = visualize_scans(scans, positions, simulator)
    
    # Visualize in polar coordinates
    fig2 = visualize_polar_scans(scans, positions)
    
    # Save the data to files
    for i, scan_data in enumerate(scans):
        filename = f'stationary_scan{i+1:03d}.csv'
        np.savetxt("scans/"+filename, scan_data, 
                  delimiter=',', header='distance,angle', comments='')
        print(f"\nSaved scan {i+1} to lidar_scan_{i+1}.csv")
    
    plt.show()
    
    # Print statistics
    print("\n" + "-" * 50)
    print("Statistics:")
    for i, scan_data in enumerate(scans):
        if len(scan_data) > 0:
            print(f"Scan {i+1}: {len(scan_data)} points, "
                  f"Min distance: {scan_data[:, 0].min():.3f}, "
                  f"Max distance: {scan_data[:, 0].max():.3f}, "
                  f"Mean distance: {scan_data[:, 0].mean():.3f}")

if __name__ == "__main__":
    main()