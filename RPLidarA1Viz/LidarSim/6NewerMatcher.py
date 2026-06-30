import numpy as np
import matplotlib.pyplot as plt
import os
import glob
from scipy.spatial import KDTree
import warnings
warnings.filterwarnings('ignore')


class RobustScanMatcher:
    def __init__(self, max_iterations=30, convergence_threshold=1e-5):
        self.max_iterations = max_iterations
        self.convergence_threshold = convergence_threshold
        self.scans = []
        self.point_clouds = []          # points in their own sensor frame
        self.aligned_clouds = []        # points transformed into world/map frame
        self.robot_trajectory = []      # (x, y, theta) poses in world frame
        self.accumulated_map = None
        self.transformations = []       # cumulative world-frame transforms

    # ------------------------------------------------------------------
    # Loading
    # ------------------------------------------------------------------

    def load_scans(self, folder_path="scans/interactive"):
        """
        Load all lidar_scan_*.csv files.

        Expected CSV columns: distance, angle
          - distance : metres (float > 0)
          - angle    : RADIANS (the simulation outputs radians, not degrees)
        """
        pattern = os.path.join(folder_path, "lidar_scan_*.csv")
        scan_files = sorted(glob.glob(pattern))

        if not scan_files:
            print(f"No scan files found in {folder_path}")
            return False

        print(f"Found {len(scan_files)} scan files")
        self.scans = []

        for file in scan_files:
            data = []
            with open(file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        parts = line.split(',')
                        if len(parts) < 2:
                            continue
                        d = float(parts[0])
                        a = float(parts[1])   # already radians — do NOT call np.radians later
                        if d > 0:             # only skip truly invalid distances
                            data.append([d, a])
                    except ValueError:
                        continue              # skip header rows or malformed lines

            if data:
                scan_data = np.array(data)
                self.scans.append(scan_data)
                print(f"  Loaded {os.path.basename(file)}: {len(scan_data)} points, "
                      f"angle range [{scan_data[:,1].min():.1f}°, {scan_data[:,1].max():.1f}°]")

        return len(self.scans) > 0

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    @staticmethod
    def polar_to_cartesian(scan_data):
        """
        Convert [distance, angle_radians] rows to (x, y) Cartesian points.
        Angles are already in radians — no conversion needed.
        """
        d = scan_data[:, 0]
        a = np.radians(scan_data[:, 1])          # radians as-is
        return np.column_stack([d * np.cos(a), d * np.sin(a)])

    @staticmethod
    def rotate_points(points, angle):
        """Rotate Nx2 array of points by `angle` radians."""
        if len(points) == 0:
            return points
        c, s = np.cos(angle), np.sin(angle)
        R = np.array([[c, -s], [s, c]])
        return points @ R.T

    @staticmethod
    def apply_transform(points, theta, tx, ty):
        """Apply SE(2) transform (theta, tx, ty) to Nx2 points."""
        return RobustScanMatcher.rotate_points(points, theta) + np.array([tx, ty])

    # ------------------------------------------------------------------
    # Preprocessing
    # ------------------------------------------------------------------

    def preprocess_scan(self, scan_data, max_distance=None, min_distance=0.05):
        """Convert and filter a raw scan. Returns Nx2 Cartesian array."""
        points = self.polar_to_cartesian(scan_data)

        norms = np.linalg.norm(points, axis=1)

        # Remove points too close to origin (sensor noise)
        mask = norms > min_distance

        # Remove points beyond max range
        if max_distance is not None:
            mask &= norms <= max_distance

        return points[mask]

    # ------------------------------------------------------------------
    # ICP core
    # ------------------------------------------------------------------

    def icp_matching(self, source_points, target_points,
                     initial_theta=0.0, initial_tx=0.0, initial_ty=0.0,
                     max_points=500):
        """
        Point-to-point ICP.  Returns (theta, tx, ty) as the SE(2) transform
        that maps source_points into the target frame, and the final aligned
        source array.

        Key fixes vs original:
          - Transform state (theta, tx, ty) is the CUMULATIVE world transform.
            Each iteration refines it; we do NOT apply the delta to `source`
            mid-loop because that would double-apply updates.
          - `matched_target` is always assigned before use.
          - No damping fudge that mixed old and new rotation in trig.
        """
        # Work on subsampled copies to keep Pi-friendly
        rng = np.random.default_rng(42)

        def subsample(pts, n):
            if len(pts) > n:
                idx = rng.choice(len(pts), n, replace=False)
                return pts[idx]
            return pts.copy()

        source = subsample(source_points, max_points)
        target = subsample(target_points, max_points)

        # Cumulative transform
        theta = float(initial_theta)
        tx    = float(initial_tx)
        ty    = float(initial_ty)

        target_tree = KDTree(target)
        prev_error  = float('inf')

        for iteration in range(self.max_iterations):
            # --- 1. Apply current cumulative transform to original source ---
            transformed = self.apply_transform(source, theta, tx, ty)

            # --- 2. Find nearest neighbours in target ---
            distances, nn_idx = target_tree.query(transformed)

            # --- 3. Reject outliers (> 3σ) — always define matched_target ---
            std_dist = np.std(distances)
            if std_dist > 1e-9:
                valid = distances < (np.mean(distances) + 3.0 * std_dist)
            else:
                valid = np.ones(len(distances), dtype=bool)

            # Ensure we keep enough points for SVD
            if valid.sum() < 10:
                valid = np.ones(len(distances), dtype=bool)

            src_valid = transformed[valid]
            tgt_valid = target[nn_idx[valid]]
            dist_valid = distances[valid]

            # --- 4. Check convergence ---
            current_error = float(np.mean(dist_valid))
            if abs(prev_error - current_error) < self.convergence_threshold:
                break
            prev_error = current_error

            # --- 5. Compute optimal delta transform via SVD ---
            src_c = np.mean(src_valid, axis=0)
            tgt_c = np.mean(tgt_valid, axis=0)

            H = (src_valid - src_c).T @ (tgt_valid - tgt_c)
            U, _, Vt = np.linalg.svd(H)
            R_delta = Vt.T @ U.T

            # Enforce proper rotation (det = +1)
            if np.linalg.det(R_delta) < 0:
                Vt[-1, :] *= -1
                R_delta = Vt.T @ U.T

            dtheta = np.arctan2(R_delta[1, 0], R_delta[0, 0])
            # Translation: move centroid of (rotated src) to centroid of tgt
            dtx = tgt_c[0] - (R_delta[0, 0] * src_c[0] + R_delta[0, 1] * src_c[1])
            dty = tgt_c[1] - (R_delta[1, 0] * src_c[0] + R_delta[1, 1] * src_c[1])

            # --- 6. Compose delta with cumulative transform ---
            # New cumulative rotation
            new_theta = theta + dtheta
            # New cumulative translation: first rotate old translation by dtheta, then add delta
            c, s = np.cos(dtheta), np.sin(dtheta)
            new_tx = c * tx - s * ty + dtx
            new_ty = s * tx + c * ty + dty

            theta, tx, ty = new_theta, new_tx, new_ty

        # Final aligned source (using full original source, not subsampled)
        aligned = self.apply_transform(source_points, theta, tx, ty)
        return (theta, tx, ty), aligned, current_error

    # ------------------------------------------------------------------
    # Initial transform estimation
    # ------------------------------------------------------------------

    def estimate_initial_transform(self, source, target):
        """
        Centroid alignment with a coarse rotation search over ±45°.
        Returns (theta, tx, ty).
        """
        src_c = np.mean(source, axis=0)
        tgt_c = np.mean(target, axis=0)

        best_theta = 0.0
        best_error = float('inf')
        target_tree = KDTree(target)

        for angle in np.linspace(-np.pi / 4, np.pi / 4, 13):
            rotated = self.rotate_points(source, angle)
            # Align centroids after rotation
            delta = tgt_c - np.mean(rotated, axis=0)
            candidate = rotated + delta
            dists, _ = target_tree.query(candidate)
            err = float(np.mean(dists))
            if err < best_error:
                best_error = err
                best_theta = angle

        # Final centroid translation for the chosen rotation
        rotated = self.rotate_points(source, best_theta)
        delta = tgt_c - np.mean(rotated, axis=0)
        return best_theta, float(delta[0]), float(delta[1])

    # ------------------------------------------------------------------
    # Map building
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Map building (FIXED)
    # ------------------------------------------------------------------

    def build_map(self, max_distance=None, min_distance=0.05):
        """
        Build a map using Scan-to-Scan matching.
        Calculates the relative motion between consecutive scans and 
        accumulates it into a global world trajectory.
        """
        if len(self.scans) < 2:
            print("Need at least 2 scans to build a map")
            return None

        print(f"\nBuilding map from {len(self.scans)} scans...")
        print("-" * 50)

        # Preprocess all scans into sensor-frame Cartesian point clouds
        self.point_clouds = []
        for i, scan in enumerate(self.scans):
            pts = self.preprocess_scan(scan, max_distance, min_distance)
            self.point_clouds.append(pts)
            print(f"  Scan {i+1}: {len(pts)} points after preprocessing")

        # --- Scan 0 defines the world frame ---
        # Pose = (theta, tx, ty) in world frame
        poses = [np.array([0.0, 0.0, 0.0])]
        self.accumulated_map = self.point_clouds[0].copy()
        self.aligned_clouds  = [self.point_clouds[0].copy()]

        print("\nICP matching (Scan-to-Scan)...")

        for i in range(1, len(self.point_clouds)):
            source = self.point_clouds[i]       # Current scan
            target = self.point_clouds[i-1]     # Previous scan

            print(f"\n  Scan {i+1} → Scan {i}")

            # Estimate initial RELATIVE transform between the two scans
            init_theta, init_tx, init_ty = self.estimate_initial_transform(source, target)

            try:
                # 1. Get the RELATIVE delta transform from i to i-1
                (d_theta, d_tx, d_ty), _, error = self.icp_matching(
                    source, target,
                    initial_theta=init_theta,
                    initial_tx=init_tx,
                    initial_ty=init_ty,
                )

                # Sanity-check: reject absurdly large relative jumps
                if abs(d_tx) > 2.0 or abs(d_ty) > 2.0:
                    print(f"    ⚠ Relative jump too large ({d_tx:.2f}, {d_ty:.2f}) — assuming 0 motion")
                    d_theta, d_tx, d_ty = 0.0, 0.0, 0.0

                # 2. Chain transforms: New World Pose = Previous World Pose * Delta
                prev_theta, prev_tx, prev_ty = poses[-1]
                
                c, s = np.cos(prev_theta), np.sin(prev_theta)
                # Rotate the relative translation into the world frame, then add to previous translation
                new_tx = prev_tx + (c * d_tx - s * d_ty)
                new_ty = prev_ty + (s * d_tx + c * d_ty)
                new_theta = prev_theta + d_theta

                poses.append(np.array([new_theta, new_tx, new_ty]))

                # 3. Transform the original source directly into the WORLD frame for mapping
                aligned_to_world = self.apply_transform(source, new_theta, new_tx, new_ty)
                
                self.aligned_clouds.append(aligned_to_world)
                self.accumulated_map = np.vstack([self.accumulated_map, aligned_to_world])

                # Keep map size manageable (only for visualization memory limits)
                if len(self.accumulated_map) > 25000:
                    idx = np.random.default_rng(i).choice(
                        len(self.accumulated_map), 20000, replace=False)
                    self.accumulated_map = self.accumulated_map[idx]

                print(f"    World Pose: θ={np.degrees(new_theta):.2f}°  "
                      f"tx={new_tx:.4f}  ty={new_ty:.4f}  mean_err={error:.5f}")

            except Exception as e:
                print(f"    ✗ ICP failed for scan {i+1}: {e}")
                # Keep map unchanged; assume the robot stopped moving
                poses.append(poses[-1].copy())
                aligned_to_world = self.apply_transform(source, *poses[-1])
                self.aligned_clouds.append(aligned_to_world)

        # Store trajectory as (x, y) pairs extracted from poses
        self.robot_trajectory = [np.array([p[1], p[2]]) for p in poses]
        self.transformations  = poses
        print("\n" + "=" * 50)
        print(f"Map building complete!")
        print(f"  Total map points : {len(self.accumulated_map)}")
        return self.accumulated_map
    
    # ------------------------------------------------------------------
    # Visualisation
    # ------------------------------------------------------------------

    def visualize_map(self):
        """Two-panel visualisation: accumulated map + per-scan overlay."""
        if self.accumulated_map is None:
            print("No map available. Run build_map() first.")
            return None

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        fig.suptitle('LiDAR Scan Matching Results', fontsize=14, fontweight='bold')

        # --- Panel 1: accumulated map + trajectory ---
        ax1.set_title('Accumulated Map with Robot Trajectory')
        ax1.scatter(self.accumulated_map[:, 0], self.accumulated_map[:, 1],
                    c='steelblue', s=1, alpha=0.4, label='Map points')

        if self.robot_trajectory:
            traj = np.array(self.robot_trajectory)
            ax1.plot(traj[:, 0], traj[:, 1], 'r-', linewidth=2, label='Trajectory')
            ax1.scatter(*traj[0],  c='limegreen', s=150, marker='*',
                        zorder=5, label='Start')
            ax1.scatter(*traj[-1], c='crimson',   s=150, marker='*',
                        zorder=5, label='End')

        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.legend(fontsize=8)

        # --- Panel 2: individual aligned scans ---
        ax2.set_title('Individual Aligned Scans')
        colours = plt.cm.plasma(np.linspace(0, 1, len(self.aligned_clouds)))

        for i, pts in enumerate(self.aligned_clouds):
            # aligned_clouds are already in world/map frame — plot directly
            label = f'Scan {i+1}' if i < 8 else ''
            ax2.scatter(pts[:, 0], pts[:, 1],
                        color=colours[i], s=2, alpha=0.5, label=label)

        ax2.set_aspect('equal')
        ax2.grid(True, alpha=0.3)
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        if len(self.aligned_clouds) <= 8:
            ax2.legend(fontsize=8, markerscale=4)

        plt.tight_layout()
        return fig

    # ------------------------------------------------------------------
    # Saving
    # ------------------------------------------------------------------

    def save_map(self, filename='built_map.csv'):
        if self.accumulated_map is not None:
            np.savetxt(filename, self.accumulated_map,
                       delimiter=',', header='x,y', comments='')
            print(f"Map saved  → {filename}")

        if self.robot_trajectory:
            traj_file = filename.replace('.csv', '_trajectory.csv')
            np.savetxt(traj_file, np.array(self.robot_trajectory),
                       delimiter=',', header='x,y', comments='')
            print(f"Trajectory → {traj_file}")

        if self.transformations:
            tf_file = filename.replace('.csv', '_transforms.csv')
            np.savetxt(tf_file, np.array(self.transformations),
                       delimiter=',', header='theta,tx,ty', comments='')
            print(f"Transforms → {tf_file}")


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

def main():
    print("=" * 60)
    print("LiDAR Scan Matcher — clean data edition")
    print("=" * 60)

    matcher = RobustScanMatcher(max_iterations=30, convergence_threshold=1e-5)

    folder_path = input(
        "Scan folder path (default: scans/interactive): ").strip()
    if not folder_path:
        folder_path = "scans/interactive"

    if not matcher.load_scans(folder_path):
        print("Failed to load scans. Check the folder path.")
        return

    print("\n" + "-" * 50)
    max_dist = input("Max range to accept (metres, default 6.0): ").strip()
    max_dist = float(max_dist) if max_dist else 6.0

    matcher.build_map(max_distance=max_dist)

    fig = matcher.visualize_map()

    save = input("\nSave map? (y/n, default y): ").strip().lower()
    if save != 'n':
        matcher.save_map('built_map.csv')

    plt.show()
    print("\nDone.")


if __name__ == "__main__":
    main()