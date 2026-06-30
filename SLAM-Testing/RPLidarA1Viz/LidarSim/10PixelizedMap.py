import glob
import os
import re
import numpy as np
from sklearn.neighbors import NearestNeighbors

# ----------------------------------------------------------------------
# Core Configuration
# ----------------------------------------------------------------------
GRID_RES_MM = 10.0  # 1cm x 1cm grid cells (10mm)


# ----------------------------------------------------------------------
# Loading & ICP Math Core (From Previous Steps)
# ----------------------------------------------------------------------

def load_scan_raw(path):
    """Loads and returns raw Nx2 (distance_mm, angle_deg) polar array."""
    polar = np.loadtxt(path, delimiter=",")
    if polar.ndim == 1:
        polar = polar.reshape(1, -1)
    return polar

def polar_to_cartesian(polar):
    dist, angle_rad = polar[:, 0], np.deg2rad(polar[:, 1])
    return np.column_stack([dist * np.cos(angle_rad), dist * np.sin(angle_rad)])

def list_scan_files(folder, pattern="real_lidar_scan_*.csv"):
    files = glob.glob(os.path.join(folder, pattern))
    def natural_sort_key(path):
        nums = re.findall(r"\d+", os.path.basename(path))
        return [int(n) for n in nums] if nums else [path]
    files.sort(key=natural_sort_key)
    return files

def best_fit_transform(A, B):
    centroid_A, centroid_B = np.mean(A, axis=0), np.mean(B, axis=0)
    H = np.dot((A - centroid_A).T, B - centroid_B)
    U, _, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(Vt.T, U.T)
    t = centroid_B.T - np.dot(R, centroid_A.T)
    T = np.identity(3)
    T[:2, :2], T[:2, 2] = R, t
    return T

def icp(A, B, max_iterations=50, tolerance=1e-5, reject_ratio=0.1):
    src = np.ones((3, A.shape[0]))
    src[:2, :] = A.T
    neigh = NearestNeighbors(n_neighbors=1).fit(B)
    prev_error = None
    for _ in range(max_iterations):
        distances, indices = neigh.kneighbors(src[:2, :].T, return_distance=True)
        distances, indices = distances.ravel(), indices.ravel()
        keep = int(len(distances) * (1 - reject_ratio)) if reject_ratio else len(distances)
        order = np.argsort(distances)[:max(keep, 4)]
        T = best_fit_transform(src[:2, order].T, B[indices[order]])
        src = np.dot(T, src)
        mean_error = np.mean(distances[order])
        if prev_error is not None and np.abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
    return best_fit_transform(A, src[:2, :].T)


# ----------------------------------------------------------------------
# Raycasting via Bresenham's Line Algorithm
# ----------------------------------------------------------------------

def bresenham_line(x0, y0, x1, y1):
    """
    Returns all grid coordinates on the line segment from (x0, y0) to (x1, y1)
    excluding the final endpoint (x1, y1).
    """
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        if x0 == x1 and y0 == y1:
            break
        points.append((x0, y0))
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
            
    return points


# ----------------------------------------------------------------------
# Grid Map Building Implementation
# ----------------------------------------------------------------------

def build_occupancy_grid(scan_files):
    # 1. First Pass: Compute Trajectory and gather all global coordinate hits
    poses = [np.identity(3)]  # Initial state: scan_0 is at (0,0,0)
    
    # Cache local Cartesian points for ICP step
    local_cartesian = [polar_to_cartesian(load_scan_raw(f)) for f in scan_files]
    
    # Propagate relative vectors sequentially to build the absolute trajectory map
    for i in range(1, len(scan_files)):
        T_rel = icp(local_cartesian[i], local_cartesian[i-1])
        poses.append(poses[-1] @ T_rel)
        
    # Gather global scan origins (robot positions) and laser endpoint hits
    all_rays = []  # format: (robot_x, robot_y, end_x, end_y)
    all_pts = []
    
    for i, f in enumerate(scan_files):
        T = poses[i]
        robot_pos = T[:2, 2]
        
        # Load raw scan data to get local endpoints
        polar = load_scan_raw(f)
        local_pts = polar_to_cartesian(polar)
        
        # Transform local endpoints into global space coordinates
        homo_pts = np.ones((local_pts.shape[0], 3))
        homo_pts[:, :2] = local_pts
        global_pts = (T @ homo_pts.T).T[:, :2]
        all_pts.append(global_pts)
        
        for pt in global_pts:
            all_rays.append((robot_pos[0], robot_pos[1], pt[0], pt[1]))

    # Flatten coordinates to determine grid dimensions bounds
    all_pts_flat = np.vstack(all_pts)
    all_poses_flat = np.array([p[:2, 2] for p in poses])
    all_coords = np.vstack([all_pts_flat, all_poses_flat])
    
    min_x, min_y = np.min(all_coords, axis=0) - 500  # padding
    max_x, max_y = np.max(all_coords, axis=0) + 500
    
    # Compute Grid dimensions
    width = int(np.ceil((max_x - min_x) / GRID_RES_MM))
    height = int(np.ceil((max_y - min_y) / GRID_RES_MM))
    
    # 0 = Unobserved/Unknown, 1 = Free Space, 2 = Occupied Wall
    grid = np.zeros((height, width), dtype=np.uint8)
    
    print(f"Generating a {width}x{height} grid map layout...")

    # Helper lambda to convert global millimeter coordinates to local discrete discrete map indices
    def to_grid(x, y):
        gx = int(np.floor((x - min_x) / GRID_RES_MM))
        gy = int(np.floor((y - min_y) / GRID_RES_MM))
        return gx, gy

    # 2. Second Pass: Apply Bresenham Raycasting
    for rx, ry, ex, ey in all_rays:
        # Convert raw millimeter positions to discrete spatial pixels indices
        gx0, gy0 = to_grid(rx, ry)
        gx1, gy1 = to_grid(ex, ey)
        
        # Guard array limits check
        if not (0 <= gx0 < width and 0 <= gy0 < height and 0 <= gx1 < width and 0 <= gy1 < height):
            continue
            
        # Trace line across free space pixels up to the barrier boundary
        free_pixels = bresenham_line(gx0, gy0, gx1, gy1)
        for fx, fy in free_pixels:
            if grid[fy, fx] != 2:  # Don't overwrite an existing confirmed wall structure
                grid[fy, fx] = 1   # Set state to Free
                
        # Mark the line trace terminal coordinate pixel point as an Occupied obstacle barrier block
        grid[gy1, gx1] = 2

    return grid, (min_x, min_y)


# ----------------------------------------------------------------------
# Map Visualization Loop
# ----------------------------------------------------------------------

if __name__ == "__main__":
    folder = os.path.join(os.path.dirname(__file__), "scans", "real")
    files = list_scan_files(folder)
    
    if not files:
        print(f"No scan data files found inside folder pattern: {folder}")
    else:
        print(f"Processing {len(files)} scan files sequentially into grid map blocks...")
        grid, origin = build_occupancy_grid(files)
        
        # Display the output metrics
        free_count = np.sum(grid == 1)
        occupied_count = np.sum(grid == 2)
        unknown_count = np.sum(grid == 0)
        
        print("\nGrid Processing Map Summary Complete:")
        print(f"  Free Space Pixels (1): {free_count}")
        print(f"  Occupied Wall Pixels (2): {occupied_count}")
        print(f"  Unknown Pixels (0): {unknown_count}")
        
        # Optional: Plot the matrix visually using standard matplotlib
        try:
            import matplotlib.pyplot as plt
            # Custom colormap color index mapping: 0 -> gray, 1 -> white, 2 -> black
            cmap = plt.cm.colors.ListedColormap(['#7f7f7f', '#ffffff', '#000000'])
            plt.figure(figsize=(10, 8))
            plt.imshow(grid, cmap=cmap, origin='lower')
            plt.title("Occupancy Grid Map (1cm resolution)")
            plt.xlabel("Grid X")
            plt.ylabel("Grid Y")
            plt.colorbar(label="0: Unknown | 1: Free | 2: Occupied")
            plt.show()
        except ImportError:
            print("\nInstall matplotlib (`pip install matplotlib`) to visually render the occupancy map result grid.")