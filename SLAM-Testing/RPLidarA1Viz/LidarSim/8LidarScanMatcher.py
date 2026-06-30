"""
Lidar scan-matching SLAM front-end.

Pipeline:
  1. Load raw (distance_mm, angle_deg) scans from CSV.
  2. Convert each scan to Cartesian points in the scan's own local sensor frame.
  3. Register consecutive scans with ICP (scan-to-scan) to recover relative
     motion (dx, dy, dtheta).
  4. Chain relative transforms into a global trajectory (pose at each scan).
  5. Transform every scan's points into the global frame using its pose and
     concatenate them into a single point-cloud map.

Conventions:
  - Distance: millimetres (as given in the raw data).
  - Angle: degrees in the raw data, converted to radians.
  - Local Cartesian frame per scan: x = forward axis, y = left axis,
    standard CCW-positive rotation (0 deg points along +x).
    angle=0 -> +x, angle=90 -> +y. This is an arbitrary but consistent
    choice; since ICP recovers the rotation between scans, the only thing
    that matters is that it's applied consistently to every scan.
  - Global frame: defined by the first scan's local frame (scan 0 sits at
    the origin with identity pose). All later poses are relative to that.
"""

import glob
import os
import re

import numpy as np
from sklearn.neighbors import NearestNeighbors


# ----------------------------------------------------------------------
# Loading & conversion
# ----------------------------------------------------------------------

def load_scan_csv(path):
    """
    Load a raw lidar scan CSV with rows: distance_mm, angle_deg
    No header row is expected (matches real_lidar_scan_x.csv format).

    Returns:
        polar: Nx2 array of (distance_mm, angle_deg)
    """
    polar = np.loadtxt(path, delimiter=",")
    if polar.ndim == 1:
        # single-row file edge case
        polar = polar.reshape(1, -1)
    return polar


def polar_to_cartesian(polar):
    """
    Convert (distance_mm, angle_deg) rows to local (x, y) Cartesian points.

    angle=0 -> +x axis, increasing angle rotates CCW toward +y.

    Input:
        polar: Nx2 array of (distance, angle_deg)
    Output:
        xy: Nx2 array of (x, y) in the same units as distance
    """
    dist = polar[:, 0]
    angle_rad = np.deg2rad(polar[:, 1])
    x = dist * np.cos(angle_rad)
    y = dist * np.sin(angle_rad)
    return np.column_stack([x, y])


def load_scan_xy(path):
    """Convenience: load a CSV straight to local-frame Cartesian points."""
    return polar_to_cartesian(load_scan_csv(path))


def natural_sort_key(path):
    """Sort 'real_lidar_scan_2.csv' before 'real_lidar_scan_10.csv'."""
    base = os.path.basename(path)
    nums = re.findall(r"\d+", base)
    return [int(n) for n in nums] if nums else [base]


def list_scan_files(folder, pattern="real_lidar_scan_*.csv"):
    files = glob.glob(os.path.join(folder, pattern))
    files.sort(key=natural_sort_key)
    return files


# ----------------------------------------------------------------------
# ICP (point-to-point, from the reference implementation)
# ----------------------------------------------------------------------

def best_fit_transform(A, B):
    """
    Least-squares best-fit transform mapping points A onto points B.

    Input:
      A: Nxm array of corresponding points
      B: Nxm array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transform mapping A onto B
      R: mxm rotation matrix
      t: mx1 translation vector
    """
    assert A.shape == B.shape
    m = A.shape[1]

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # reflection correction
    if np.linalg.det(R) < 0:
        Vt[m - 1, :] *= -1
        R = np.dot(Vt.T, U.T)

    t = centroid_B.T - np.dot(R, centroid_A.T)

    T = np.identity(m + 1)
    T[:m, :m] = R
    T[:m, m] = t
    return T, R, t


def nearest_neighbor(src, dst):
    """Find nearest neighbor in dst for each point in src."""
    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()


def icp(A, B, init_pose=None, max_iterations=50, tolerance=1e-5,
        reject_ratio=None, return_history=False):
    """
    Iterative Closest Point: find the transform mapping A onto B.

    A and B may have DIFFERENT numbers of points (this is the normal case
    for two independent lidar scans) -- only matched pairs are used inside
    the loop, best_fit_transform always receives equal-length matched sets.

    Input:
        A: Nxm array of source points (will be moved)
        B: Mxm array of destination/reference points (stays fixed)
        init_pose: (m+1)x(m+1) initial homogeneous transform guess
        max_iterations: cap on ICP iterations
        tolerance: stop when mean-error improvement falls below this
        reject_ratio: if set (e.g. 0.1), drop the worst this-fraction of
            correspondences each iteration by distance, before fitting.
            Helps with non-overlapping regions between scans.
        return_history: if True, also return the list of intermediate
            source-point-cloud snapshots (for animating convergence).

    Output:
        T: final homogeneous transform mapping A onto B
        distances: final per-point nearest-neighbor distances
        i: number of iterations run
        history: list of Nxm arrays (only if return_history=True)
    """
    assert A.shape[1] == B.shape[1]
    m = A.shape[1]

    src = np.ones((m + 1, A.shape[0]))
    dst = np.ones((m + 1, B.shape[0]))
    src[:m, :] = A.T.copy()
    dst[:m, :] = B.T.copy()

    if init_pose is not None:
        src = np.dot(init_pose, src)

    history = [src[:m, :].T.copy()] if return_history else None

    prev_error = None
    distances = None
    for i in range(max_iterations):
        distances, indices = nearest_neighbor(src[:m, :].T, dst[:m, :].T)

        if reject_ratio:
            keep = int(len(distances) * (1 - reject_ratio))
            keep = max(keep, m + 2)  # need enough points to fit a transform
            order = np.argsort(distances)[:keep]
        else:
            order = np.arange(len(distances))

        T, _, _ = best_fit_transform(src[:m, order].T, dst[:m, indices[order]].T)
        src = np.dot(T, src)

        if return_history:
            history.append(src[:m, :].T.copy())

        mean_error = np.mean(distances[order])
        if prev_error is not None and np.abs(prev_error - mean_error) < tolerance:
            prev_error = mean_error
            break
        prev_error = mean_error

    T, _, _ = best_fit_transform(A, src[:m, :].T)

    if return_history:
        return T, distances, i, history
    return T, distances, i


# ----------------------------------------------------------------------
# Pose chaining
# ----------------------------------------------------------------------

def transform_points(T, pts):
    """Apply a homogeneous transform T to Nx2 points."""
    m = pts.shape[1]
    homo = np.ones((pts.shape[0], m + 1))
    homo[:, :m] = pts
    out = (T @ homo.T).T
    return out[:, :m]


def pose_to_xytheta(T):
    """Extract (x, y, theta_rad) from a 3x3 homogeneous 2D transform."""
    x, y = T[0, 2], T[1, 2]
    theta = np.arctan2(T[1, 0], T[0, 0])
    return x, y, theta


def build_trajectory(scan_files, max_iterations=50, tolerance=1e-5,
                      reject_ratio=0.1, init_with_prev_motion=True):
    """
    Run scan-to-scan ICP across a sequence of scan files and chain the
    relative transforms into a global trajectory.

    Returns a dict with:
      'files': list of scan file paths (in order)
      'local_points': list of Nx2 arrays, local-frame Cartesian points per scan
      'global_points': list of Nx2 arrays, points transformed into global frame
      'poses': list of 3x3 homogeneous transforms (scan-frame -> global-frame)
      'relative_transforms': list of 3x3 transforms (scan i -> scan i-1), len = n-1
      'icp_info': list of dicts with per-pair {'mean_error', 'iterations'}
    """
    local_points = [load_scan_xy(f) for f in scan_files]

    n = len(local_points)
    poses = [np.identity(3)]  # scan 0 defines the global frame
    relative_transforms = []
    icp_info = []

    prev_rel = np.identity(3)  # constant-velocity init guess

    for i in range(1, n):
        A = local_points[i]       # source: current scan
        B = local_points[i - 1]   # destination: previous scan

        init_guess = prev_rel if init_with_prev_motion else None
        T, distances, iters = icp(
            A, B,
            init_pose=init_guess,
            max_iterations=max_iterations,
            tolerance=tolerance,
            reject_ratio=reject_ratio,
        )
        relative_transforms.append(T)
        icp_info.append({"mean_error": float(np.mean(distances)), "iterations": int(iters)})

        global_pose = poses[-1] @ T
        poses.append(global_pose)

        prev_rel = T

    global_points = [transform_points(poses[i], local_points[i]) for i in range(n)]

    return {
        "files": scan_files,
        "local_points": local_points,
        "global_points": global_points,
        "poses": poses,
        "relative_transforms": relative_transforms,
        "icp_info": icp_info,
    }

import matplotlib.pyplot as plt

def visualize_slam_results(result):
    """
    Plots the final SLAM map and trajectory alongside a side-by-side comparison
    of the local scans vs. global alignment.
    """
    # Extract trajectory positions (x, y)
    trajectory = np.array([pose_to_xytheta(T)[:2] for T in result["poses"]])
    
    # Concatenate all global points into a single map
    all_global_points = np.vstack(result["global_points"])
    
    # Setup a clean 1x2 subplot layout
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
    fig.suptitle("Lidar Scan-Matching SLAM Results", fontsize=16, fontweight='bold')
    
    # ------------------------------------------------------------------
    # Subplot 1: The Global Map and Trajectory
    # ------------------------------------------------------------------
    # Plot the registered global map points (downsampled slightly with alpha for performance/clarity)
    ax1.scatter(all_global_points[:, 0], all_global_points[:, 1], 
                s=1, c='black', alpha=0.5, label='Mapped Environment')
    
    # Plot the robot's trajectory path
    ax1.plot(trajectory[:, 0], trajectory[:, 1], 
             color='crimson', linewidth=2, marker='o', markersize=4, label='Robot Trajectory')
    
    # Highlight the Start and End points
    ax1.scatter(trajectory[0, 0], trajectory[0, 1], color='green', s=100, zorder=5, label='Start')
    ax1.scatter(trajectory[-1, 0], trajectory[-1, 1], color='orange', s=100, zorder=5, label='End')
    
    ax1.set_title("Global Point Cloud Map & Trajectory")
    ax1.set_xlabel("X Position (mm)")
    ax1.set_ylabel("Y Position (mm)")
    ax1.axis('equal')
    ax1.grid(True, linestyle='--', alpha=0.5)
    ax1.legend(loc='upper right')
    
    # ------------------------------------------------------------------
    # Subplot 2: Local Scan Overlap vs Global Registration
    # ------------------------------------------------------------------
    # To demonstrate registration, let's overlay the first two scans in their 
    # original local frames vs their registered global frames
    if len(result["local_points"]) > 1:
        scan0_local = result["local_points"][0]
        scan1_local = result["local_points"][1]
        scan1_global = result["global_points"][1]
        
        # Plot local unaligned frames (offsetting scan 1 slightly just to show raw difference if they overlap tightly)
        ax2.scatter(scan0_local[:, 0], scan0_local[:, 1], s=3, color='blue', alpha=0.5, label='Scan 0 (Ref)')
        ax2.scatter(scan1_local[:, 0], scan1_local[:, 1], s=3, color='red', alpha=0.3, label='Scan 1 (Raw Local)')
        ax2.scatter(scan1_global[:, 0], scan1_global[:, 1], s=3, color='green', alpha=0.8, label='Scan 1 (ICP Aligned)')
        
        ax2.set_title("Scan Registration: Raw Local vs. ICP Global")
        ax2.set_xlabel("X Position (mm)")
        ax2.set_ylabel("Y Position (mm)")
        ax2.axis('equal')
        ax2.grid(True, linestyle='--', alpha=0.5)
        ax2.legend(loc='upper right')
    else:
        ax2.text(0.5, 0.5, "Need at least 2 scans\nto show registration.", 
                 ha='center', va='center', fontsize=12)
        ax2.set_title("Scan Registration")

    plt.tight_layout()
    plt.show()

# ----------------------------------------------------------------------
# Execution hook update
# ----------------------------------------------------------------------


if __name__ == "__main__":
    folder = os.path.join(os.path.dirname(__file__), "scans", "real")
    files = list_scan_files(folder)
    print(f"Found {len(files)} scan files:")
    for f in files:
        print(" ", os.path.basename(f))

    result = build_trajectory(files)
    print("\nTrajectory (global pose per scan):")
    for f, T in zip(result["files"], result["poses"]):
        x, y, theta = pose_to_xytheta(T)
        print(f"  {os.path.basename(f)}: x={x:.1f}mm  y={y:.1f}mm  theta={np.rad2deg(theta):.2f}deg")

    print("\nICP fit quality per pair:")
    for i, info in enumerate(result["icp_info"]):
        print(f"  scan {i} -> scan {i+1}: mean_error={info['mean_error']:.2f}mm  iters={info['iterations']}")

    folder = os.path.join(os.path.dirname(__file__), "scans", "real")
    files = list_scan_files(folder)
    result = build_trajectory(files)
    
    # Fire up the visualization window!
    #visualize_slam_results(result)