#!/usr/bin/env python3
"""

Frontier goals aren't safety-checked against the inflation mask. 
update_goal_to_nearest_frontier picks the first free cell it finds adjacent to unknown space via BFS, 
with no regard for whether that cell is inside your SAFETY_RADIUS_CELLS buffer around a nearby wall. 
get_neighbors then explicitly exempts the goal cell from the safety check (by design 
— otherwise frontier goals near walls would be unreachable). Net effect: your robot could occasionally be sent to hug a wall.
 If that matters for your hardware, one option is to have the frontier search itself skip candidate cells
   that fall inside self._inflated_obstacles, computed once at the top of map_callback before the BFS runs 
   (right now inflation is only built after the frontier is chosen).
"""
# ODREDJUJE GDE TREBA ICI OD REZULTATA MAPE
from collections import deque

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import heapq
import math
import numpy as np
from scipy import ndimage

SAFETY_RADIUS_CELLS = 5
OBSTACLE_THRESHOLD = 70   # occupancy value at/above which a cell is a wall


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # --- Pretplacen na mapu ---
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10
        )

        # --- Objavljuje put ---
        # FIX: was '11' (looks like a leftover placeholder). motion_planner_node
        # subscribes to 'global_path', so nothing downstream ever received a
        # path before this fix, regardless of whether A* succeeded.
        self.path_pub = self.create_publisher(
            Path, '/global_path', 10
        )

        # --- Pretplacen na poziciju robota (Odom ili Slam) ---
        self.pose_sub = self.create_subscription(
            Odometry, '/odom', self.pose_callback, 10
        )

        # Koordinate koje se menjaju usput, pocetak na globalnoj 0
        self.start_x = 0.0
        self.start_y = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0

        # Precomputed per-map-update obstacle inflation (see map_callback)
        self._inflated_obstacles = None

        self.get_logger().info("A* pokrenut")

    def update_goal_to_nearest_frontier(self, msg):
        # TRAZI NAJBLIZI NEPOSECEN PROSTOR KOJI JE DOSTUPAN IZ SLOBODNOG PROSTORA
        width = msg.info.width
        height = msg.info.height

        start_grid = self.world_to_grid(self.start_x, self.start_y, msg.info)

        # Ako je robot izvan granica, ne pokreci BFS
        if not (0 <= start_grid[0] < width and 0 <= start_grid[1] < height):
            return False

        # FIX: was `queue.pop(0)` on a plain list, which is O(n) per pop and
        # turns this BFS into O(n^2) on a large grid. deque.popleft() is O(1).
        queue = deque([start_grid])
        visited = {start_grid}

        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 4 glavna smera

        while queue:
            cx, cy = queue.popleft()
            current_idx = cy * width + cx

            # Provera da li smo nasli nepoznatu celiju (-1) koja se granici sa slobodnim prostorom
            if msg.data[current_idx] == -1:
                has_free_neighbor = False
                for dx, dy in directions:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        n_idx = ny * width + nx
                        if msg.data[n_idx] == 0:
                            has_free_neighbor = True
                            wx, wy = self.grid_to_world(nx, ny, msg.info)
                            self.goal_x = wx
                            self.goal_y = wy
                            self.get_logger().info(f"Pronadjena granica! Novi cilj na slobodnom prostoru: ({wx:.2f}, {wy:.2f})")
                            return True

            if msg.data[current_idx] == 0:
                for dx, dy in directions:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        queue.append((nx, ny))

        self.get_logger().info("Mapiranje potpuno zavrseno! Nema vise dostupnog neistrazenog prostora.")
        return False

    def pose_callback(self, msg):
        self.start_x = msg.pose.pose.position.x
        self.start_y = msg.pose.pose.position.y

    def map_callback(self, msg):
        self.get_logger().info("Obradjuje mapu za A*", throttle_duration_sec=4.0)

        has_frontier = self.update_goal_to_nearest_frontier(msg)
        if not has_frontier:
            self.get_logger().info("Ulazi u standby")
            return

        start_grid = self.world_to_grid(self.start_x, self.start_y, msg.info)
        goal_grid = self.world_to_grid(self.goal_x, self.goal_y, msg.info)

        if not (0 <= start_grid[0] < msg.info.width and 0 <= start_grid[1] < msg.info.height):
            self.get_logger().error("Robot start position is outside the current map boundaries!")
            return

        # FIX: precompute obstacle inflation ONCE per map update instead of
        # doing an 11x11 nested-loop scan for every single neighbor expansion
        # inside A*. This is both a large speedup and a correctness fix --
        # see _build_inflated_obstacles for the bug it also fixes.
        self._inflated_obstacles = self._build_inflated_obstacles(msg)

        grid_path = self.a_star(start_grid, goal_grid, msg)

        if not grid_path:
            self.get_logger().warn("A* failed to find a valid path!")
            return

        path_msg = Path()
        path_msg.header.frame_id = msg.header.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for cell in grid_path:
            world_x, world_y = self.grid_to_world(cell[0], cell[1], msg.info)
            pose = PoseStamped()
            pose.header.frame_id = msg.header.frame_id
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def _build_inflated_obstacles(self, msg):
        """Builds a boolean grid where True means 'too close to (or on) a
        wall to drive through'.

        FIX: the original per-neighbor check only ran the wall-clearance
        scan when a cell's OWN occupancy value was in [0, 40). A cell whose
        occupancy was >= 40 -- including an actual wall marked 100 -- never
        entered that branch and fell through to `is_safe = True`, meaning
        A* could treat real obstacle cells as safe to path through. Using
        a proper morphological dilation of the obstacle mask fixes this:
        the dilated mask always includes the original obstacle cells
        themselves (dilation only ever grows a mask, never shrinks it), so
        both "on a wall" and "too close to a wall" are excluded correctly,
        in one O(1)-per-cell lookup instead of an O(radius^2) scan per
        neighbor.
        """
        grid_np = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        obstacle_mask = grid_np >= OBSTACLE_THRESHOLD

        r = SAFETY_RADIUS_CELLS
        yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
        disk = (xx ** 2 + yy ** 2) <= r ** 2

        return ndimage.binary_dilation(obstacle_mask, structure=disk)

    def world_to_grid(self, wx, wy, geo):
        # FIX: was `int(...)`, which truncates toward zero instead of
        # flooring. For any coordinate that comes out negative relative to
        # the map origin (e.g. a slightly stale /odom reading right after
        # the map has grown and shifted its origin), truncation rounds the
        # wrong way and lands one cell off. slam_mapping_node already uses
        # floor for the same conversion -- this matches it.
        gx = int(math.floor((wx - geo.origin.position.x) / geo.resolution))
        gy = int(math.floor((wy - geo.origin.position.y) / geo.resolution))
        return (gx, gy)

    def grid_to_world(self, gx, gy, geo):
        wx = geo.origin.position.x + (gx + 0.5) * geo.resolution
        wy = geo.origin.position.y + (gy + 0.5) * geo.resolution
        return (wx, wy)

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def get_neighbors(self, node, msg, goal=None):
        width = msg.info.width
        height = msg.info.height
        neighbors = []

        directions = [
            (0, 1, 1.0), (1, 0, 1.0), (0, -1, 1.0), (-1, 0, 1.0),
            (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)
        ]

        for dx, dy, cost in directions:
            nx, ny = node[0] + dx, node[1] + dy

            if 0 <= nx < width and 0 <= ny < height:
                # Goal cell is allowed to skip the wall-clearance check: by
                # definition it sits next to unknown (-1) space (it's a
                # frontier cell), so enforcing clearance from -1 there would
                # make the goal permanently unreachable.
                is_goal_cell = (goal is not None and (nx, ny) == goal)

                is_safe = is_goal_cell or not self._inflated_obstacles[ny, nx]

                # FIX: diagonal steps were only checked against the diagonal
                # target cell itself, not the two orthogonal cells the robot
                # would have to squeeze between. That let A* cut through the
                # corner of a wall as if there were a gap, when the two
                # flanking cells are actually blocked. Require both
                # orthogonal neighbors of a diagonal step to be clear too.
                if is_safe and not is_goal_cell and dx != 0 and dy != 0:
                    corner_blocked = (
                        self._inflated_obstacles[node[1], nx]
                        or self._inflated_obstacles[ny, node[0]]
                    )
                    if corner_blocked:
                        is_safe = False

                if is_safe:
                    neighbors.append(((nx, ny), cost))

        return neighbors

    def a_star(self, start, goal, msg):
        open_heap = []
        heapq.heappush(open_heap, (0.0, start))
        # FIX: original membership test was
        #   `if not any(item[1] == neighbor for item in open_set)`
        # which linearly scans the entire heap for every single neighbor
        # relaxation -- O(n) per check, making the whole search effectively
        # O(n^2) on any non-trivial grid, which reads as "A* just hangs".
        # A plain set gives O(1) membership checks instead.
        open_set = {start}
        closed_set = set()
        came_from = {}
        g_score = {start: 0.0}
        f_score = {start: self.heuristic(start, goal)}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            open_set.discard(current)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            if current in closed_set:
                continue
            closed_set.add(current)

            for neighbor, step_cost in self.get_neighbors(current, msg, goal):
                if neighbor in closed_set:
                    continue

                tentative_g_score = g_score[current] + step_cost

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)

                    if neighbor not in open_set:
                        heapq.heappush(open_heap, (f_score[neighbor], neighbor))
                        open_set.add(neighbor)
        return None


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()