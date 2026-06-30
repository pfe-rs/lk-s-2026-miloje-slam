import glob
import os
import re
import numpy as np
from sklearn.neighbors import NearestNeighbors

# ----------------------------------------------------------------------
# Loading & conversion
# ----------------------------------------------------------------------

def load_scan_xy(path):
    """Load raw CSV and convert (distance_mm, angle_deg) straight to Cartesian."""
    polar = np.loadtxt(path, delimiter=",")
    if polar.ndim == 1:
        polar = polar.reshape(1, -1)
    
    dist, angle_rad = polar[:, 0], np.deg2rad(polar[:, 1])
    return np.column_stack([dist * np.cos(angle_rad), dist * np.sin(angle_rad)])

def list_scan_files(folder, pattern="real_lidar_scan_*.csv"):
    """Find and naturally sort scan files."""
    files = glob.glob(os.path.join(folder, pattern))
    def natural_sort_key(path):
        nums = re.findall(r"\d+", os.path.basename(path))
        return [int(n) for n in nums] if nums else [path]
    files.sort(key=natural_sort_key)
    return files

# ----------------------------------------------------------------------
# ICP Math Core
# ----------------------------------------------------------------------

def best_fit_transform(A, B):
    """Least-squares best-fit transform mapping 2D points A onto points B."""
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
    """Iterative Closest Point mapping point cloud A onto reference B."""
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
# Relative Transformation Extraction
# ----------------------------------------------------------------------

def calculate_relative_transforms(scan_files, reject_ratio=0.1):
    """
    Processes scan files sequentially to compute relative transforms between steps.
    Returns a list of dicts containing the file pairs and [dx, dy, dtheta].
    """
    scans = [load_scan_xy(f) for f in scan_files]
    relative_transforms = []
    
    for i in range(1, len(scans)):
        T = icp(scans[i], scans[i-1], reject_ratio=reject_ratio)
        
        dx = T[0, 2]
        dy = T[1, 2]
        dtheta = np.arctan2(T[1, 0], T[0, 0])
        
        relative_transforms.append({
            "from_file": os.path.basename(scan_files[i]),
            "to_file": os.path.basename(scan_files[i-1]),
            "vector": [dx, dy, dtheta]
        })
        
    return relative_transforms

# ----------------------------------------------------------------------
# Execution Hook
# ----------------------------------------------------------------------

if __name__ == "__main__":
    # Point this to your scans directory
    folder = os.path.join(os.path.dirname(__file__), "scans", "real")
    files = list_scan_files(folder)
    
    print(f"Found {len(files)} scan files. Processing relative transformations...\n")
    
    rel_transforms = calculate_relative_transforms(files)
    
    print(f"{'Source Scan':<25} -> {'Reference Scan':<25} | {'dx (mm)':<10} {'dy (mm)':<10} {'dtheta (deg)':<12}")
    print("-" * 90)
    for transform in rel_transforms:
        dx, dy, dtheta = transform["vector"] # dx, dy, theta
        print(f"{transform['from_file']:<25} -> {transform['to_file']:<25} | {dx:<10.1f} {dy:<10.1f} {np.rad2deg(dtheta):<12.2f}")