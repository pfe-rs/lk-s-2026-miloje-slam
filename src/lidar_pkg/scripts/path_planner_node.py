#!/usr/bin/env python3
# ODREDJUJE GDE TREBA ICI OD REZULTATA MAPE
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import heapq
import math

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # --- Pretplacen na mapu ---
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10
        )
        

        # --- Objavljuje put ---
        self.path_pub = self.create_publisher(
            Path, '11', 10
        )

        # --- Pretplacen na poziciju robota (Odom ili Slam) ---
        
        self.pose_sub = self.create_subscription(
            Odometry, 'odom', self.pose_callback, 10
        )

        # Koordinate koje se menjaju usput, pocetak na globalnoj 0
        self.start_x = 0.0
        self.start_y = 0.0
        self.goal_x = 0.0 
        self.goal_y = 0.0

        self.get_logger().info("A* pokrenut")

    def update_goal_to_nearest_frontier(self, msg):
        # TRAZI NAJBLIZI NEPOSECEN PROSTOR KOJI JE DOSTUPAN IZ SLOBODNOG PROSTORA
        width = msg.info.width
        height = msg.info.height
        
        start_grid = self.world_to_grid(self.start_x, self.start_y, msg.info)
        
        # Ako je robot izvan granica, ne pokreci BFS
        if not (0 <= start_grid[0] < width and 0 <= start_grid[1] < height):
            return False
            
        queue = [start_grid]
        visited = {start_grid}
        
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)] # 4 glavna smera
        
        while queue:
            cx, cy = queue.pop(0)
            current_idx = cy * width + cx
            
            # Provera da li smo nasli nepoznatu celiju (-1) koja se granici sa slobodnim prostorom
            if msg.data[current_idx] == -1:
                # Proveri da li je bar jedan komsija zapravo slobodan prostor (vrednost 0)
                # To osigurava da robot moze da stigne DO te granice
                has_free_neighbor = False
                for dx, dy in directions:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        n_idx = ny * width + nx
                        if msg.data[n_idx] == 0: # 0 znaci cist, slobodan prostor iz nase slam mape
                            has_free_neighbor = True
                            # Postavljamo cilj na ovaj slobodan komsijski kvadrat da A* ne pukne
                            wx, wy = self.grid_to_world(nx, ny, msg.info)
                            self.goal_x = wx
                            self.goal_y = wy
                            self.get_logger().info(f"Pronadjena granica! Novi cilj na slobodnom prostoru: ({wx:.2f}, {wy:.2f})")
                            return True
                
            # Ako je trenutna celija poznat slobodan prostor (0), sirimo pretragu dalje
            if msg.data[current_idx] == 0:
                for dx, dy in directions:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < width and 0 <= ny < height and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        queue.append((nx, ny))
                        
        self.get_logger().info("Mapiranje potpuno zavrseno! Nema vise dostupnog neistrazenog prostora.")
        return False
 
    def pose_callback(self, msg):
        #APDEJT POZICIJE
        self.start_x = msg.pose.pose.position.x
        self.start_y = msg.pose.pose.position.y

    def map_callback(self, msg):
        self.get_logger().info("Obradjuje mapu za A*", throttle_duration_sec=4.0)

        # TRAZI NOVU METU KOD GRANICE POZNATOG/NEPOZNATOG
        has_frontier = self.update_goal_to_nearest_frontier(msg)
        if not has_frontier:
            self.get_logger().info("Ulazi u standby")
            return # GOTOV ULAZI U STANDBY

        # POKRECE A* DO CILJA
        start_grid = self.world_to_grid(self.start_x, self.start_y, msg.info)
        goal_grid = self.world_to_grid(self.goal_x, self.goal_y, msg.info)

        # IZBEGAVANJE CVOROVA VAN MAPE
        if not (0 <= start_grid[0] < msg.info.width and 0 <= start_grid[1] < msg.info.height):
            self.get_logger().error("Robot start position is outside the current map boundaries!")
            return

        grid_path = self.a_star(start_grid, goal_grid, msg)

        if not grid_path:
            self.get_logger().warn("A* failed to find a valid path!")
            return

        # KONVERZIJA PUTA
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

    def world_to_grid(self, wx, wy, geo):
        gx = int((wx - geo.origin.position.x) / geo.resolution)
        gy = int((wy - geo.origin.position.y) / geo.resolution)
        return (gx, gy)

    def grid_to_world(self, gx, gy, geo):
        wx = geo.origin.position.x + (gx + 0.5) * geo.resolution
        wy = geo.origin.position.y + (gy + 0.5) * geo.resolution
        return (wx, wy)

    def heuristic(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

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
                index = ny * width + nx
                
                # Index protection guard
                if index >= len(msg.data):
                    continue
                    
                occupancy_value = msg.data[index]

                # AKO JE PREBLIZU ZIDA, IZBEGNI
                is_safe = True
                safety_radius = 5

                # Goal cell is allowed to skip the wall-clearance check: by
                # definition it sits next to unknown (-1) space (it's a
                # frontier cell), so enforcing clearance from -1 there would
                # make the goal permanently unreachable.
                is_goal_cell = (goal is not None and (nx, ny) == goal)

                if 0 <= occupancy_value < 40 and not is_goal_cell: # Lowered tolerance for obstacles
                    # Quick check around the neighbor cell to ensure no walls are too close
                    for sx in range(-safety_radius, safety_radius + 1):
                        for sy in range(-safety_radius, safety_radius + 1):
                            check_x, check_y = nx + sx, ny + sy
                            if 0 <= check_x < width and 0 <= check_y < height:
                                check_idx = check_y * width + check_x
                                # Only treat actual obstacles (>70) as unsafe.
                                # Unknown cells (-1) are expected near a
                                # frontier goal and must not block expansion.
                                if msg.data[check_idx] > 70:
                                    is_safe = False
                                    break
                        if not is_safe: break

                if is_safe:
                    neighbors.append(((nx, ny), cost))

        return neighbors

    def a_star(self, start, goal, msg):
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for neighbor, step_cost in self.get_neighbors(current, msg, goal):
                tentative_g_score = g_score[current] + step_cost

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)

                    if not any(item[1] == neighbor for item in open_set):
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
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