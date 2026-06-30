import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from rplidar import RPLidar
from scipy.optimize import minimize

PORT_NAME = '/dev/ttyUSB0'   
MAX_DISTANCE = 8000  # Limited to 8m for A1 to reduce noise at max range     

lidar = RPLidar(PORT_NAME)

# Threading storage
latest_scan = []
data_lock = threading.Lock()
is_running = True

# --- SLAM State Variables ---
global_map_points = np.empty((0, 2))
previous_scan_points = None
global_pose = np.array([0.0, 0.0, 0.0])  # [x, y, theta] in global frame

# --- NDT & Map Parameters ---
NDT_CELL_SIZE = 400     # Smaller cells (400mm) for scan-to-scan precision
MAP_VOXEL_SIZE = 100    # Downsample the map to 1 point per 100x100mm area

def lidar_background_worker():
    """Reads LiDAR scans in the background."""
    global latest_scan, is_running
    try:
        for scan in lidar.iter_scans(max_buf_meas=500):
            if not is_running:
                break
            with data_lock:
                latest_scan = scan
    except Exception as e:
        print(f"Error reading LiDAR: {e}")

def polar_to_cartesian(scan):
    """Converts RPLidar polar coordinates to Cartesian (x, y)."""
    angles = np.radians([s[1] for s in scan])
    distances = np.array([s[2] for s in scan])
    
    # Filter out 0 distances and extremely far noisy points
    valid = (distances > 50) & (distances < MAX_DISTANCE)
    angles = angles[valid]
    distances = distances[valid]
    
    x = distances * np.cos(angles)
    y = distances * np.sin(angles)
    return np.column_stack((x, y))

def compose_poses(pose1, pose2):
    """
    Combines two 2D poses: calculates the new global pose given a previous global pose and a local movement.
    pose1: global_pose [X, Y, Theta]
    pose2: delta_pose  [dx, dy, dtheta]
    """
    x1, y1, th1 = pose1
    x2, y2, th2 = pose2
    
    c, s = np.cos(th1), np.sin(th1)
    
    # Matrix multiplication for 2D transformation
    x_new = x1 + c * x2 - s * y2
    y_new = y1 + s * x2 + c * y2
    th_new = th1 + th2
    
    return np.array([x_new, y_new, th_new])

def downsample_map(points, voxel_size):
    """Spatial hashing to keep only one point per voxel area, preventing memory bloat."""
    if len(points) == 0:
        return points
    
    # Convert points to integer grid indices
    indices = np.floor(points / voxel_size).astype(int)
    
    # Keep only the unique grid coordinates (effectively downsampling)
    _, unique_indices = np.unique(indices, axis=0, return_index=True)
    return points[unique_indices]

# ----------------- NDT Algorithm -----------------

def build_ndt_map(points, cell_size):
    """Builds normal distributions for a 2D grid."""
    grid = {}
    for p in points:
        idx = (int(p[0] // cell_size), int(p[1] // cell_size))
        if idx not in grid:
            grid[idx] = []
        grid[idx].append(p)
        
    ndt_map = {}
    for idx, pts in grid.items():
        if len(pts) >= 3:
            pts_arr = np.array(pts)
            mean = np.mean(pts_arr, axis=0)
            cov = np.cov(pts_arr, rowvar=False) + np.eye(2) * 1e-3 # Epsilon to prevent singular matrix
            
            try:
                inv_cov = np.linalg.inv(cov)
                ndt_map[idx] = {'mean': mean, 'inv_cov': inv_cov}
            except np.linalg.LinAlgError:
                continue
            
    return ndt_map

def transform_points(points, params):
    """Applies 2D rotation and translation to a set of points."""
    tx, ty, theta = params
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return np.dot(points, R.T) + np.array([tx, ty])

def ndt_objective(params, moving_points, ndt_map, cell_size):
    """Objective function: Maximize probability (minimize negative probability)."""
    transformed = transform_points(moving_points, params)
    score = 0.0
    
    for p in transformed:
        idx = (int(p[0] // cell_size), int(p[1] // cell_size))
        if idx in ndt_map:
            mu = ndt_map[idx]['mean']
            inv_cov = ndt_map[idx]['inv_cov']
            diff = p - mu
            
            # Probability evaluation
            exponent = -0.5 * np.dot(diff.T, np.dot(inv_cov, diff))
            score += np.exp(exponent)
            
    return -score 

def match_ndt(target, source, cell_size):
    """Aligns source points to target points using NDT with movement bounds."""
    ndt_map = build_ndt_map(target, cell_size)
    if not ndt_map:
        return np.array([0.0, 0.0, 0.0])
        
    init_guess = np.array([0.0, 0.0, 0.0])
    
    # We restrict the optimizer bounds: between two frames, the robot cannot physically
    # move more than 300mm or rotate more than ~28 degrees (0.5 rad).
    # This specifically stops the scans from rotating backwards/spinning out of control.
    bounds = [(-300, 300), (-300, 300), (-0.5, 0.5)]
    
    res = minimize(ndt_objective, init_guess, args=(source, ndt_map, cell_size), 
                   method='Powell', bounds=bounds)
    return res.x

# ----------------- Visualizer Setup -----------------
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_aspect('equal')
ax.set_xlim(-MAX_DISTANCE, MAX_DISTANCE)
ax.set_ylim(-MAX_DISTANCE, MAX_DISTANCE)
ax.set_title("Real-Time NDT SLAM (Map Accumulation)")

scatter_global = ax.scatter([], [], c='blue', s=2, alpha=0.3, label='Global Map')
scatter_current = ax.scatter([], [], c='red', s=4, label='Current Scan')
plt.legend()

def update_plot(frame):
    global latest_scan, global_map_points, previous_scan_points, global_pose
    
    with data_lock:
        current_scan_raw = list(latest_scan)
        
    if len(current_scan_raw) < 20:
        return scatter_global, scatter_current
        
    current_points = polar_to_cartesian(current_scan_raw)
    
    # Initialization Step
    if previous_scan_points is None:
        previous_scan_points = current_points
        global_map_points = transform_points(current_points, global_pose)
        scatter_global.set_offsets(global_map_points)
        return scatter_global, scatter_current
        
    # 1. Match current scan to previous scan to find the small delta movement
    delta_pose = match_ndt(previous_scan_points, current_points, NDT_CELL_SIZE)
    
    # 2. Update the robot's global position using the delta
    global_pose = compose_poses(global_pose, delta_pose)
    
    # 3. Transform the current scan into the global coordinate system
    current_points_global = transform_points(current_points, global_pose)
    
    # 4. Append the new points to the global map and downsample
    global_map_points = np.vstack((global_map_points, current_points_global))
    global_map_points = downsample_map(global_map_points, MAP_VOXEL_SIZE)
    
    # 5. Update visualizations
    scatter_global.set_offsets(global_map_points)
    scatter_current.set_offsets(current_points_global)
    
    # Update state for the next frame
    previous_scan_points = current_points
        
    return scatter_global, scatter_current

def on_close(event):
    """Ensures the LiDAR motor stops spinning when the window is closed."""
    global is_running
    print("Shutting down LiDAR...")
    is_running = False
    worker_thread.join()
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

# Start the LiDAR background thread
worker_thread = threading.Thread(target=lidar_background_worker, daemon=True)
worker_thread.start()

# Bind the close event and start the animation
fig.canvas.mpl_connect('close_event', on_close)
ani = FuncAnimation(fig, update_plot, interval=200, blit=True)

plt.show()