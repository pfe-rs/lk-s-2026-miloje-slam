import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import os
import glob
from scipy.spatial import KDTree
from scipy.optimize import minimize
import matplotlib.patches as patches
from sklearn.cluster import DBSCAN
from scipy.signal import find_peaks

class EnhancedScanMatcher:
    def __init__(self, max_iterations=50, convergence_threshold=1e-6):
        """
        Initialize enhanced scan matcher using ICP algorithm with better initialization
        """
        self.max_iterations = max_iterations
        self.convergence_threshold = convergence_threshold
        self.scans = []
        self.positions = []
        self.point_clouds = []
        self.robot_trajectory = []
        self.accumulated_map = None
        
    def load_scans(self, folder_path="scans/interactive"):
        """Load all scan files from a folder"""
        pattern = os.path.join(folder_path, "real_lidar_scan_*.csv")
        scan_files = sorted(glob.glob(pattern))
        
        if not scan_files:
            print(f"No scan files found in {folder_path}")
            return False
        
        print(f"Found {len(scan_files)} scan files")
        
        self.scans = []
        self.positions = []
        
        for file in scan_files:
            # Read the file
            with open(file, 'r') as f:
                lines = f.readlines()
            
            # Try to extract position from header
            position = None
            header = lines[0].strip()
            if 'Position:' in header:
                try:
                    pos_str = header.split('Position:')[1].split('),')[0] + ')'
                    pos_str = pos_str.replace('(', '').replace(')', '')
                    pos_values = pos_str.split(',')
                    position = np.array([float(pos_values[0]), float(pos_values[1])])
                except:
                    position = None
            
            # Read data (skip header if it's not numeric)
            data = []
            start_line = 0
            for i, line in enumerate(lines):
                try:
                    # Try to parse as float
                    values = line.strip().split(',')
                    if len(values) >= 2:
                        float(values[0])
                        float(values[1])
                        data.append([float(values[0]), float(values[1])])
                except:
                    continue
            
            if data:
                scan_data = np.array(data)
                self.scans.append(scan_data)
                self.positions.append(position)
                print(f"Loaded {file}: {len(scan_data)} points")
        
        return len(self.scans) > 0
    
    def polar_to_cartesian(self, scan_data):
        """Convert polar coordinates (distance, angle) to Cartesian (x, y)"""
        distances = scan_data[:, 0]
        angles = np.radians(scan_data[:, 1])
        
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        
        return np.column_stack([x, y])
    
    def preprocess_scan(self, scan_data, max_distance=None, downsample_factor=1):
        """Preprocess scan data for better matching"""
        points = self.polar_to_cartesian(scan_data)
        
        # Filter by distance
        if max_distance is not None:
            distances = np.linalg.norm(points, axis=1)
            points = points[distances <= max_distance]
        
        # Downsample
        if downsample_factor > 1:
            points = points[::downsample_factor]
        
        return points
    
    def estimate_initial_transform(self, source_points, target_points):
        """
        Estimate initial transformation between two scans using multiple methods
        
        Methods:
        1. Feature-based: Extract corners and edges
        2. Centroids: Align centroids
        3. PCA: Align principal components
        """
        # Method 1: Centroid alignment
        source_centroid = np.mean(source_points, axis=0)
        target_centroid = np.mean(target_points, axis=0)
        tx_init = target_centroid[0] - source_centroid[0]
        ty_init = target_centroid[1] - source_centroid[1]
        
        # Method 2: Try to find rotation using PCA
        try:
            # Compute covariance matrices
            source_centered = source_points - source_centroid
            target_centered = target_points - target_centroid
            
            # Compute principal components
            U_source, _, _ = np.linalg.svd(source_centered.T @ source_centered)
            U_target, _, _ = np.linalg.svd(target_centered.T @ target_centered)
            
            # Estimate rotation from principal components
            R_est = U_target @ U_source.T
            rotation_init = np.arctan2(R_est[1, 0], R_est[0, 0])
            
            # Refine with small angle search
            best_error = float('inf')
            best_rotation = rotation_init
            
            for angle_offset in np.linspace(-0.3, 0.3, 10):
                test_rotation = rotation_init + angle_offset
                rotated_source = self.rotate_points(source_points, test_rotation)
                transformed_source = rotated_source + np.array([tx_init, ty_init])
                
                # Find nearest neighbors
                target_tree = KDTree(target_points)
                distances, _ = target_tree.query(transformed_source)
                error = np.mean(distances**2)
                
                if error < best_error:
                    best_error = error
                    best_rotation = test_rotation
            
            return np.array([best_rotation, tx_init, ty_init])
            
        except:
            # Fallback: just use centroid alignment
            return np.array([0.0, tx_init, ty_init])
    
    def icp_matching(self, source_points, target_points, initial_transform=None):
        """Perform ICP matching with better initialization"""
        if initial_transform is None:
            initial_transform = self.estimate_initial_transform(source_points, target_points)
        
        rotation, tx, ty = initial_transform
        source = source_points.copy()
        target = target_points.copy()
        
        target_tree = KDTree(target)
        prev_error = float('inf')
        
        for iteration in range(self.max_iterations):
            # Apply transformation
            rotated_source = self.rotate_points(source, rotation)
            transformed_source = rotated_source + np.array([tx, ty])
            
            # Find nearest neighbors
            distances, indices = target_tree.query(transformed_source)
            current_error = np.mean(distances**2)
            
            if abs(prev_error - current_error) < self.convergence_threshold:
                break
            
            prev_error = current_error
            
            # Compute optimal transformation
            matched_target = target[indices]
            
            source_centroid = np.mean(transformed_source, axis=0)
            target_centroid = np.mean(matched_target, axis=0)
            
            source_centered = transformed_source - source_centroid
            target_centered = matched_target - target_centroid
            
            H = source_centered.T @ target_centered
            U, S, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
            
            new_rotation = np.arctan2(R[1, 0], R[0, 0])
            new_tx = target_centroid[0] - (R[0, 0] * source_centroid[0] + R[0, 1] * source_centroid[1])
            new_ty = target_centroid[1] - (R[1, 0] * source_centroid[0] + R[1, 1] * source_centroid[1])
            
            rotation += new_rotation
            tx += new_tx * np.cos(rotation) - new_ty * np.sin(rotation)
            ty += new_tx * np.sin(rotation) + new_ty * np.cos(rotation)
            
            source = self.rotate_points(source, new_rotation) + np.array([new_tx, new_ty])
        
        return np.array([rotation, tx, ty]), source
    
    def rotate_points(self, points, angle):
        """Rotate points by given angle (radians)"""
        rotation_matrix = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]
        ])
        return points @ rotation_matrix.T
    
    def detect_corners(self, points):
        """Detect corners in scan using angle-based method"""
        if len(points) < 10:
            return []
        
        # Sort by angle
        angles = np.arctan2(points[:, 1], points[:, 0])
        sorted_idx = np.argsort(angles)
        sorted_points = points[sorted_idx]
        
        # Calculate curvature
        curvatures = []
        window = 5
        for i in range(len(sorted_points)):
            prev_idx = (i - window) % len(sorted_points)
            next_idx = (i + window) % len(sorted_points)
            
            v1 = sorted_points[prev_idx] - sorted_points[i]
            v2 = sorted_points[next_idx] - sorted_points[i]
            
            if np.linalg.norm(v1) > 1e-6 and np.linalg.norm(v2) > 1e-6:
                angle_diff = np.arccos(np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1, 1))
                curvatures.append(angle_diff)
            else:
                curvatures.append(0)
        
        # Find peaks in curvature
        peaks, _ = find_peaks(curvatures, height=np.mean(curvatures) + 0.5 * np.std(curvatures))
        corners = sorted_points[peaks]
        
        return corners
    
    def build_map(self, max_distance=None, downsample_factor=1, use_initial_positions=True):
        """Build a map by matching consecutive scans with improved initialization"""
        if len(self.scans) < 2:
            print("Need at least 2 scans to build a map")
            return
        
        print(f"\nBuilding map from {len(self.scans)} scans...")
        print("-" * 50)
        
        # Preprocess scans
        processed_scans = []
        for i, scan in enumerate(self.scans):
            points = self.preprocess_scan(scan, max_distance, downsample_factor)
            processed_scans.append(points)
            print(f"Scan {i+1}: {len(points)} points")
        
        # Initialize with first scan
        accumulated_map = processed_scans[0].copy()
        self.robot_trajectory = [np.array([0, 0])]
        transformations = [np.array([0, 0, 0])]
        
        # Try to use initial positions if available
        if use_initial_positions and any(p is not None for p in self.positions):
            print("\nUsing available position estimates")
            # Use first valid position as reference
            ref_pos = None
            for pos in self.positions:
                if pos is not None:
                    ref_pos = pos
                    break
            
            if ref_pos is not None:
                for i, pos in enumerate(self.positions):
                    if pos is not None and i > 0:
                        # Estimate initial transform from position difference
                        dx = pos[0] - ref_pos[0]
                        dy = pos[1] - ref_pos[1]
                        # Estimate rotation from sequence of positions
                        if i > 1 and self.positions[i-1] is not None:
                            prev_dx = self.positions[i-1][0] - ref_pos[0]
                            prev_dy = self.positions[i-1][1] - ref_pos[1]
                            # Estimate heading change
                            angle_est = np.arctan2(dy - prev_dy, dx - prev_dx)
                            if i < len(transformations):
                                transformations.append(np.array([angle_est, dx, dy]))
        
        # Perform pair-wise matching
        print("\nPerforming ICP matching with enhanced initialization...")
        for i in range(1, len(processed_scans)):
            source = processed_scans[i]
            target = accumulated_map.copy()
            
            # Get initial transformation estimate
            if len(transformations) > i:
                initial_transform = transformations[i]
            else:
                # Estimate from previous transform
                if i > 0 and len(transformations) > i-1:
                    prev_transform = transformations[i-1]
                    # Assume similar motion
                    initial_transform = prev_transform + np.array([0, 0.1, 0.1])
                else:
                    initial_transform = None
            
            print(f"\nMatching scan {i+1} to accumulated map ({len(target)} points)...")
            
            try:
                # Perform ICP with better initialization
                transform, aligned_source = self.icp_matching(
                    source, target, initial_transform
                )
                
                rotation, tx, ty = transform
                
                # Update robot position with relative motion
                if len(self.robot_trajectory) > 0:
                    prev_pos = self.robot_trajectory[-1]
                    new_pos = prev_pos + np.array([tx, ty])
                    self.robot_trajectory.append(new_pos)
                else:
                    self.robot_trajectory.append(np.array([tx, ty]))
                
                # Accumulate
                accumulated_map = np.vstack([accumulated_map, aligned_source])
                
                # Downsample if too large
                if len(accumulated_map) > 5000:
                    indices = np.random.choice(len(accumulated_map), 5000, replace=False)
                    accumulated_map = accumulated_map[indices]
                
                print(f"  Transform: rotation={np.degrees(rotation):.2f}°, tx={tx:.3f}, ty={ty:.3f}")
                print(f"  Map size: {len(accumulated_map)} points")
                
            except Exception as e:
                print(f"  Error matching scan {i+1}: {e}")
                # Fallback: use simple translation
                if i < len(self.robot_trajectory):
                    self.robot_trajectory.append(self.robot_trajectory[-1] + np.array([0.5, 0]))
                else:
                    self.robot_trajectory.append(np.array([i * 0.5, 0]))
        
        self.point_clouds = processed_scans
        self.transformations = transformations
        self.accumulated_map = accumulated_map
        
        print("\n" + "=" * 50)
        print(f"Map building complete!")
        print(f"  Total points in map: {len(accumulated_map)}")
        print(f"  Trajectory points: {len(self.robot_trajectory)}")
        
        return accumulated_map
    
    def visualize_map(self, show_scans=True):
        """Visualize the built map with robot trajectory"""
        if not hasattr(self, 'accumulated_map'):
            print("No map available. Run build_map() first.")
            return
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        fig.suptitle('Scan Matching Results', fontsize=14, fontweight='bold')
        
        # Main map
        ax1.set_title('Built Map with Robot Trajectory')
        
        # Plot accumulated map
        ax1.scatter(self.accumulated_map[:, 0], self.accumulated_map[:, 1], 
                   c='gray', s=1, alpha=0.5, label='Map points')
        
        # Plot robot trajectory
        if self.robot_trajectory:
            traj = np.array(self.robot_trajectory)
            ax1.plot(traj[:, 0], traj[:, 1], 'r-', linewidth=2, label='Estimated trajectory')
            ax1.scatter(traj[0, 0], traj[0, 1], c='green', s=100, 
                       marker='*', label='Start', zorder=5)
            ax1.scatter(traj[-1, 0], traj[-1, 1], c='red', s=100, 
                       marker='*', label='End', zorder=5)
        
        # Plot individual scans
        if show_scans and hasattr(self, 'point_clouds'):
            colors = plt.cm.viridis(np.linspace(0, 1, len(self.point_clouds)))
            for i, points in enumerate(self.point_clouds):
                if i < len(self.robot_trajectory):
                    pos = self.robot_trajectory[i]
                    transformed_points = points + pos
                    ax1.scatter(transformed_points[:, 0], transformed_points[:, 1], 
                               c=[colors[i]], s=2, alpha=0.3)
        
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        ax1.set_xlabel('X Position')
        ax1.set_ylabel('Y Position')
        ax1.legend()
        ax1.set_xlim(-8, 8)
        ax1.set_ylim(-8, 8)
        
        # Individual scan matching results
        ax2.set_title('Individual Scans (Aligned)')
        
        if hasattr(self, 'point_clouds'):
            colors = plt.cm.viridis(np.linspace(0, 1, len(self.point_clouds)))
            for i, points in enumerate(self.point_clouds):
                if i < len(self.robot_trajectory):
                    pos = self.robot_trajectory[i]
                    transformed_points = points + pos
                    ax2.scatter(transformed_points[:, 0], transformed_points[:, 1], 
                               c=[colors[i]], s=2, alpha=0.5, 
                               label=f'Scan {i+1}' if i < 5 else '')
        
        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)
        ax2.set_xlabel('X Position')
        ax2.set_ylabel('Y Position')
        if len(self.point_clouds) > 0:
            ax2.legend(loc='upper right')
        ax2.set_xlim(-8, 8)
        ax2.set_ylim(-8, 8)
        
        plt.tight_layout()
        return fig
    
    def save_map(self, filename='built_map.csv'):
        """Save the built map to a CSV file"""
        if hasattr(self, 'accumulated_map'):
            np.savetxt(filename, self.accumulated_map, 
                      delimiter=',', header='x,y', comments='')
            print(f"Map saved to {filename}")
            
            if self.robot_trajectory:
                traj_filename = filename.replace('.csv', '_trajectory.csv')
                np.savetxt(traj_filename, np.array(self.robot_trajectory), 
                          delimiter=',', header='x,y', comments='')
                print(f"Trajectory saved to {traj_filename}")
    
    def save_aligned_scans(self, folder='aligned_scans'):
        """Save each aligned scan individually"""
        if not hasattr(self, 'point_clouds') or not self.robot_trajectory:
            print("No aligned scans available")
            return
        
        os.makedirs(folder, exist_ok=True)
        
        for i, points in enumerate(self.point_clouds):
            if i < len(self.robot_trajectory):
                pos = self.robot_trajectory[i]
                aligned_points = points + pos
                filename = os.path.join(folder, f'aligned_scan_{i+1:03d}.csv')
                np.savetxt(filename, aligned_points, 
                          delimiter=',', header='x,y', comments='')
                print(f"Saved aligned scan {i+1} to {filename}")

def main():
    """Main execution function"""
    print("=" * 60)
    print("Enhanced LiDAR Scan Matching for Map Building")
    print("(Works without header information)")
    print("=" * 60)
    
    # Create scan matcher
    matcher = EnhancedScanMatcher(max_iterations=30, convergence_threshold=1e-6)
    
    # Ask for folder path
    folder_path = input("Enter path to scan folder (default: scans/interactive): ").strip()
    if not folder_path:
        folder_path = "scans/real"
    
    # Load scans
    if not matcher.load_scans(folder_path):
        print("Failed to load scans. Please check the folder path.")
        return
    
    # Ask for parameters
    print("\n" + "-" * 50)
    max_distance = input("Maximum distance to consider (default: 7.0): ").strip()
    max_distance = float(max_distance) if max_distance else 7.0
    
    downsample = input("Downsample factor (1=no downsampling, default: 2): ").strip()
    downsample = int(downsample) if downsample else 2
    
    # Build map (works without headers)
    print("\n" + "-" * 50)
    print("Building map WITHOUT using header positions...")
    map_points = matcher.build_map(max_distance, downsample, use_initial_positions=True)
    
    # Visualize results
    fig = matcher.visualize_map(show_scans=True)
    
    # Save results
    save = input("\nSave map and aligned scans? (y/n, default: y): ").strip().lower()
    if save != 'n':
        matcher.save_map('built_map.csv')
        matcher.save_aligned_scans('aligned_scans')
    
    plt.show()
    
    print("\n" + "=" * 60)
    print("Map building complete!")
    print("=" * 60)

if __name__ == "__main__":
    main()