import time
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from rplidar import RPLidar
from scipy.spatial import KDTree

PORT_NAME = '/dev/ttyUSB0'
MAX_DISTANCE = 10000

lidar = RPLidar(PORT_NAME)
#lidar._serial_port.setDTR(False)
time.sleep(2)

latest_scan = []
data_lock = threading.Lock()
is_running = True

ICP_ITERATIONS = 20
ICP_TOLERANCE = 0.1
MAX_CORRESPONDENCE_DIST = 200
MAX_MAP_POINTS = 5000
MIN_ICP_POINTS = 50

global_map = None
current_pose_R = np.eye(2)
current_pose_t = np.zeros(2)
last_displacement = np.zeros(3)

def lidar_background_worker():
    global latest_scan, is_running
    try:
        for scan in lidar.iter_scans():
            if not is_running:
                break
            with data_lock:
                latest_scan = scan
    except Exception as e:
        print(f"LiDAR error: {e}")
        is_running = False


worker_thread = threading.Thread(target=lidar_background_worker, daemon=True)
worker_thread.start()


# --- COORDINATE CONVERSION ---

def polarToCartesian(scan_data):
    """
    Converts raw RPLidar scan tuples to Cartesian (N, 2).

    RPLidar gives angles in DEGREES, measured CLOCKWISE from forward.
    We convert to standard math convention (counterclockwise from X-axis)
    by negating the angle before applying trig — this is the key fix that
    prevents the coordinate frame mismatch that caused the spiral.

    scan_data: list of (quality, angle_deg, distance_mm) tuples
    """
    angles_deg = np.array([item[1] for item in scan_data])
    distances  = np.array([item[2] for item in scan_data])

    # FIX: Negate angle to convert CW -> CCW convention
    angles_rad = np.radians(-angles_deg)

    x = np.cos(angles_rad) * distances
    y = np.sin(angles_rad) * distances
    return np.column_stack((x, y))


# --- ICP ENGINE ---

def findCorrespondences(map_points, scan_points):
    tree = KDTree(map_points)
    distances, indices = tree.query(scan_points)
    inlier_mask = distances < MAX_CORRESPONDENCE_DIST
    if np.sum(inlier_mask) < MIN_ICP_POINTS:
        return None, None
    return scan_points[inlier_mask], map_points[indices[inlier_mask]]


def estimateTransform(source, target):
    c_src = np.mean(source, axis=0)
    c_tgt = np.mean(target, axis=0)
    H = (source - c_src).T @ (target - c_tgt)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T
    t = c_tgt - R @ c_src
    return R, t


def applyTransform(points, R, t):
    return (R @ points.T).T + t


def rotationAngle(R):
    return np.arctan2(R[1, 0], R[0, 0])


def subsampleMap(points, voxel_size=30):
    """
    Snaps points to a grid (e.g., 30mm) and keeps only unique voxels.
    This prevents blurry walls and stops ICP rotational drift.
    """
    if len(points) == 0:
        return points
    voxels = np.round(points / voxel_size).astype(int)
    _, unique_indices = np.unique(voxels, axis=0, return_index=True)
    return points[unique_indices]


def runICP(map_points, new_scan, initial_R=None, initial_t=None):
    """
    Aligns a local scan to the global map.

    Returns:
        aligned_scan: scan points transformed into map coordinates
        pose_R: rotation from scan frame to map frame
        pose_t: translation from scan frame to map frame
        converged: whether ICP stopped by tolerance
    """
    if initial_R is None:
        initial_R = np.eye(2)
    if initial_t is None:
        initial_t = np.zeros(2)

    # Downsample the map to 50mm for faster/stable KDTree matching
    map_sub = subsampleMap(map_points, voxel_size=50)
    pose_R = np.array(initial_R, copy=True)
    pose_t = np.array(initial_t, copy=True)
    aligned = applyTransform(new_scan, pose_R, pose_t)
    
    for i in range(ICP_ITERATIONS):
        src, tgt = findCorrespondences(map_sub, aligned)
        if src is None:
            print(f"ICP: too few correspondences at iteration {i}")
            return None, None, None, False

        delta_R, delta_t = estimateTransform(src, tgt)
        aligned_new = applyTransform(aligned, delta_R, delta_t)
        movement = np.mean(np.linalg.norm(aligned_new - aligned, axis=1))
        aligned = aligned_new

        # Compose the incremental ICP correction with the current scan-to-map pose.
        pose_R = delta_R @ pose_R
        pose_t = delta_R @ pose_t + delta_t

        if movement < ICP_TOLERANCE:
            return aligned, pose_R, pose_t, True
            
    print("ICP: max iterations reached")
    return aligned, pose_R, pose_t, False


def processScan(scan_data):
    """
    Matches a new RPLidar scan against the existing map, updates the map,
    and returns the robot displacement since the previous accepted scan.

    Returns displacement as [dx_mm, dy_mm, dtheta_rad] in the global map frame.
    """
    global global_map, current_pose_R, current_pose_t, last_displacement

    current_scan = [(q, a, d) for q, a, d in scan_data if 0 < d <= MAX_DISTANCE]
    if len(current_scan) < MIN_ICP_POINTS:
        return None, None

    new_scan_cart = polarToCartesian(current_scan)

    if global_map is None:
        global_map = subsampleMap(np.copy(new_scan_cart), voxel_size=30)
        current_pose_R = np.eye(2)
        current_pose_t = np.zeros(2)
        last_displacement = np.zeros(3)
        print("Initial map established.")
        return np.copy(new_scan_cart), np.copy(last_displacement)

    previous_pose_R = np.copy(current_pose_R)
    previous_pose_t = np.copy(current_pose_t)

    aligned, pose_R, pose_t, converged = runICP(
        global_map,
        new_scan_cart,
        initial_R=current_pose_R,
        initial_t=current_pose_t,
    )

    if aligned is None:
        return applyTransform(new_scan_cart, current_pose_R, current_pose_t), None

    if not converged:
        print("ICP: using best available alignment")

    current_pose_R = pose_R
    current_pose_t = pose_t

    displacement_xy = current_pose_t - previous_pose_t
    displacement_theta = rotationAngle(current_pose_R @ previous_pose_R.T)
    last_displacement = np.array([
        displacement_xy[0],
        displacement_xy[1],
        displacement_theta,
    ])

    global_map = np.vstack((global_map, aligned))
    global_map = subsampleMap(global_map, voxel_size=30)

    return aligned, np.copy(last_displacement)

# --- VISUALISATION (Cartesian XY — no polar plot) ---
# FIX: Use a plain Cartesian axes instead of polar.
# This removes the polar <-> Cartesian round-trip that introduced
# per-frame rounding error and made debugging harder.

fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-MAX_DISTANCE, MAX_DISTANCE)
ax.set_ylim(-MAX_DISTANCE, MAX_DISTANCE)
ax.set_aspect('equal')
ax.set_facecolor('black')
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_title('LiDAR SLAM — Cartesian view')

map_line,  = ax.plot([], [], '.', color='dodgerblue', markersize=1, label='Global Map')
scan_line, = ax.plot([], [], '.', color='red',        markersize=2, label='Current Scan')
ax.legend(loc='upper right')


def update(frame):
    global latest_scan, global_map

    with data_lock:
        scan_snapshot = list(latest_scan)

    new_scan_cart, displacement = processScan(scan_snapshot)
    if new_scan_cart is None:
        return map_line, scan_line

    if displacement is not None:
        print(
            "Displacement: "
            f"dx={displacement[0]:.1f} mm, "
            f"dy={displacement[1]:.1f} mm, "
            f"dtheta={np.degrees(displacement[2]):.2f} deg"
        )

    map_line.set_data(global_map[:, 0], global_map[:, 1])
    scan_line.set_data(new_scan_cart[:, 0], new_scan_cart[:, 1])

    return map_line, scan_line

try:
    print("LiDAR SLAM active (Cartesian display)...")
    ani = FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
    plt.show()

except KeyboardInterrupt:
    print("Stopping...")

finally:
    is_running = False
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    print("Cleanup done.")
