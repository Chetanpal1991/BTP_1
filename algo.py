import math
from heapq import heappush, heappop

from obstacle_creator import ObstacleMapManager


def dist(a, b):
    """Calculate Euclidean distance between two points"""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def manhattan_dist(a, b):
    """Calculate Manhattan distance for A* heuristic"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def is_valid_position(x, y, obstacles, grid_size, robot_size=1):
    """Check if a position is valid (within bounds and not overlapping obstacles)"""
    for dx in range(robot_size):
        for dy in range(robot_size):
            px, py = x + dx, y + dy
            # Check bounds for each cell
            if not (0 <= px < grid_size and 0 <= py < grid_size):
                return False
            # Check if position overlaps with any obstacle
            if (px, py) in obstacles:
                return False
    return True


def get_neighbors(pos, obstacles, grid_size):
    """Get valid neighboring positions with diagonal movement restrictions"""
    x, y = pos
    neighbors = []

    # 8-directional movement with diagonal restrictions
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue

            new_x, new_y = x + dx, y + dy

            # Check if the target position is valid
            if not is_valid_position(new_x, new_y, obstacles, grid_size):
                continue

            # For diagonal moves, check if both adjacent orthogonal cells are free
            if dx != 0 and dy != 0:  # This is a diagonal move
                # Check the two adjacent orthogonal positions
                orthogonal1 = (x + dx, y)      # horizontal adjacent
                orthogonal2 = (x, y + dy)      # vertical adjacent

                # Both orthogonal positions must be valid for diagonal move to be allowed
                if (is_valid_position(orthogonal1[0], orthogonal1[1], obstacles, grid_size) and
                    is_valid_position(orthogonal2[0], orthogonal2[1], obstacles, grid_size)):
                    neighbors.append((new_x, new_y))
            else:
                # Straight moves (orthogonal) are always allowed if target is valid
                neighbors.append((new_x, new_y))

    return neighbors


def a_star(start, goal, obstacles, grid_size=55):
    """
    A* pathfinding algorithm.
    
    Returns a list of (x, y) tuples representing the path from start to goal.
    Returns empty list if no path exists.
    """
    # Priority queue for open set: (f_score, position)
    open_set = []
    heappush(open_set, (0, start))

    # Dictionary to reconstruct path
    came_from = {}

    # Cost from start to each position
    g_score = {start: 0}

    # Estimated total cost from start to goal through each position
    f_score = {start: manhattan_dist(start, goal)}

    while open_set:
        # Get position with lowest f_score
        current = heappop(open_set)[1]

        # Check if we've reached the goal
        if current == goal:
            # Reconstruct and return path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Reverse to get path from start to goal

        # Explore neighbors
        for neighbor in get_neighbors(current, obstacles, grid_size):
            # Calculate tentative g_score for this neighbor
            tentative_g_score = g_score[current] + dist(current, neighbor)

            # If this path to neighbor is better than any previous one
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                # Record this path
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + manhattan_dist(neighbor, goal)

                # Add neighbor to open set if not already there
                heappush(open_set, (f_score[neighbor], neighbor))

    return []  # No path found


class AStarPlanner:
    """
    Hybrid A* + APF planner for a single robot.

    This planner:
    - Computes A* path ONCE for global planning
    - Switches to APF mode when other robots are nearby
    - Returns to A* path when the way is clear
    - Uses 8-directional movement (N, NE, E, SE, S, SW, W, NW)
    """

    def __init__(self, obstacle_manager, robot_config, grid_size=55, **params):
        """
        Args:
            obstacle_manager: obstacle manager reference
            robot_config: {'start': (x,y), 'goal': (x,y)}
            grid_size: int
            **params: vision_radius, k_att, k_rep, rep_radius
        """
        self.obstacle_manager = obstacle_manager
        self.robot_config = robot_config
        self.grid_size = grid_size

        # A* path tracking
        self.path = []
        self.path_index = 0
        self.at_goal = False

        # APF parameters
        self.vision_radius = params.get('vision_radius', 5.0)
        self.k_att = params.get('k_att', 1.0)
        self.k_rep = params.get('k_rep', 2.0)
        self.rep_radius = params.get('rep_radius', 3.0)

        # 8 directions: N, NE, E, SE, S, SW, W, NW
        self.directions = [
            (0, 1),   # N
            (1, 1),   # NE
            (1, 0),   # E
            (1, -1),  # SE
            (0, -1),  # S
            (-1, -1), # SW
            (-1, 0),  # W
            (-1, 1)   # NW
        ]

    def compute_all_paths(self):
        """
        Compute A* path for this single robot.
        Returns a dict: {'R': path}
        """
        obstacles = self.obstacle_manager.get_obstacles()
        start = self.robot_config['start']
        goal = self.robot_config['goal']

        print("\nComputing A* path for single robot...")

        path = a_star(start, goal, obstacles, self.grid_size)

        if not path:
            print(f"  ✗ No path found ({start} → {goal})")
            path = [start]
        else:
            print(f"  ✓ Path found: {len(path)} steps")

        self.path = path
        self.path_index = 0
        self.at_goal = (len(path) == 1 or start == goal)

        return {'R': self.path.copy()}

    def update_positions(self, current_positions):
        """
        Hybrid update: use APF if peers nearby, otherwise follow A* path.

        current_positions = {'R': (x, y), 'peers': [(x1,y1), (x2,y2), ...]}
        
        Returns:
            {'R': (next_x, next_y)}
        """
        obstacles = self.obstacle_manager.get_obstacles()
        current_pos = current_positions['R']
        cx, cy = current_pos

        # Check if at goal
        if self.at_goal:
            return {'R': current_pos}

        goal = self.robot_config['goal']
        if (int(cx), int(cy)) == goal:
            self.at_goal = True
            return {'R': current_pos}

        # Get peer robots in vision
        peers = current_positions.get('peers', [])

        # Decide mode: APF if peers nearby, otherwise A*
        if peers:
            # APF MODE
            next_pos = self._apf_step(current_pos, peers, obstacles, goal)
        else:
            # A* MODE
            next_pos = self._astar_step(current_pos, obstacles, current_positions)

        return {'R': next_pos}

    def _astar_step(self, current_pos, obstacles, all_positions):
        """Follow A* path step-by-step."""
        cx, cy = current_pos

        # Check if path exhausted
        if self.path_index >= len(self.path) - 1:
            self.at_goal = True
            return current_pos

        # Get next waypoint
        next_cell = self.path[self.path_index + 1]
        nx, ny = next_cell

        # Check if blocked by other robots
        for name, pos in all_positions.items():
            if name == 'R' or name == 'peers':
                continue
            px, py = int(pos[0]), int(pos[1])
            if (px, py) == (nx, ny):
                return current_pos  # wait

        # Check if blocked by obstacle
        if not is_valid_position(nx, ny, obstacles, self.grid_size):
            return current_pos

        # Move forward
        self.path_index += 1
        return (float(nx), float(ny))

    def _apf_step(self, current_pos, peers, obstacles, goal):
        """Use APF to compute next move based on peers and obstacles."""
        cx, cy = current_pos

        # Determine target: next A* waypoint or goal
        if self.path_index < len(self.path) - 1:
            target = self.path[self.path_index + 1]
        else:
            target = goal

        # Compute attractive force toward target
        f_att = self._attractive_force(current_pos, target)

        # Compute repulsive forces from peers
        f_rep_peers = self._repulsive_force_peers(current_pos, peers)

        # Compute repulsive forces from obstacles
        f_rep_obs = self._repulsive_force_obstacles(current_pos, obstacles)

        # Total force
        fx = f_att[0] + f_rep_peers[0] + f_rep_obs[0]
        fy = f_att[1] + f_rep_peers[1] + f_rep_obs[1]

        # Convert force vector to 8-directional movement
        direction = self._vector_to_direction(fx, fy)

        # Try to move in that direction
        next_pos = self._try_move(current_pos, direction, obstacles, peers)

        # If moved successfully and APF mode ended, snap back to A* path
        if not peers and next_pos != current_pos:
            self._snap_to_path(next_pos)

        return next_pos

    def _attractive_force(self, pos, target):
        """Compute attractive force toward target."""
        dx = target[0] - pos[0]
        dy = target[1] - pos[1]
        distance = math.hypot(dx, dy)
        
        if distance < 0.1:
            return (0.0, 0.0)
        
        # Normalized direction scaled by k_att
        fx = self.k_att * dx / distance
        fy = self.k_att * dy / distance
        return (fx, fy)

    def _repulsive_force_peers(self, pos, peers):
        """Compute repulsive force from peer robots."""
        fx_total, fy_total = 0.0, 0.0

        for peer_pos in peers:
            dx = pos[0] - peer_pos[0]
            dy = pos[1] - peer_pos[1]
            distance = math.hypot(dx, dy)

            if distance < 0.1:
                distance = 0.1  # Avoid division by zero

            if distance < self.rep_radius:
                magnitude = self.k_rep * (1.0/distance - 1.0/self.rep_radius) / (distance**2)
                fx_total += magnitude * dx / distance
                fy_total += magnitude * dy / distance

        return (fx_total, fy_total)

    def _repulsive_force_obstacles(self, pos, obstacles):
        """Compute repulsive force from obstacles."""
        fx_total, fy_total = 0.0, 0.0

        for obs in obstacles:
            dx = pos[0] - obs[0]
            dy = pos[1] - obs[1]
            distance = math.hypot(dx, dy)

            if distance < 0.1:
                distance = 0.1

            if distance < self.rep_radius:
                magnitude = self.k_rep * (1.0/distance - 1.0/self.rep_radius) / (distance**2)
                fx_total += magnitude * dx / distance
                fy_total += magnitude * dy / distance

        return (fx_total, fy_total)

    def _vector_to_direction(self, fx, fy):
        """Convert force vector to nearest 8-direction."""
        if abs(fx) < 0.01 and abs(fy) < 0.01:
            return (0, 0)  # No movement

        angle = math.atan2(fy, fx)
        
        # Map angle to 8 directions (0°=E, 45°=NE, 90°=N, etc.)
        # Divide circle into 8 sectors of 45° each
        sector = int(round(angle / (math.pi / 4))) % 8
        
        # Map sectors to directions
        direction_map = [
            (1, 0),   # 0° - E
            (1, 1),   # 45° - NE
            (0, 1),   # 90° - N
            (-1, 1),  # 135° - NW
            (-1, 0),  # 180° - W
            (-1, -1), # 225° - SW
            (0, -1),  # 270° - S
            (1, -1)   # 315° - SE
        ]
        
        return direction_map[sector]

    def _try_move(self, pos, preferred_dir, obstacles, peers):
        """Try to move in preferred direction, fallback to nearby directions."""
        cx, cy = pos

        if preferred_dir == (0, 0):
            return pos  # No movement suggested

        # Find index of preferred direction
        try:
            preferred_idx = self.directions.index(preferred_dir)
        except ValueError:
            preferred_idx = 0

        # Test directions in order of angular closeness
        for offset in [0, 1, -1, 2, -2, 3, -3, 4]:
            idx = (preferred_idx + offset) % 8
            dx, dy = self.directions[idx]
            nx, ny = int(cx + dx), int(cy + dy)

            # Check bounds
            if not (0 <= nx < self.grid_size and 0 <= ny < self.grid_size):
                continue

            # Check obstacles
            if not is_valid_position(nx, ny, obstacles, self.grid_size):
                continue

            # Check peers
            blocked = False
            for peer_pos in peers:
                if (int(peer_pos[0]), int(peer_pos[1])) == (nx, ny):
                    blocked = True
                    break

            if not blocked:
                return (float(nx), float(ny))

        # All directions blocked - stay in place
        return pos

    def _snap_to_path(self, current_pos):
        """Snap back to nearest point on A* path after APF mode."""
        if not self.path or self.at_goal:
            return

        # Find closest path index
        min_dist = float('inf')
        best_idx = self.path_index

        for i in range(self.path_index, len(self.path)):
            d = dist(current_pos, self.path[i])
            if d < min_dist:
                min_dist = d
                best_idx = i

        self.path_index = best_idx

robots_config = {
            'R1': {'start': (0, 0), 'goal': (54, 54)},
            'R2': {'start': (0, 54), 'goal': (54, 0)},
            # 'R3': {'start': (54, 0), 'goal': (0, 54)},
            # 'R4': {'start': (54, 54), 'goal': (0, 0)}
        }



obstacle_manager = ObstacleMapManager(
            grid_size=55,
            save_file="yielding_robot_obstacle_maps.json"
        )

obstacles = set(obstacle_manager.get_obstacles())
print("data type of obstacles : " + str(type(obstacles)))



print("data type of global path : " + str(type(a_star((0, 0), (54, 54), obstacles))))

trial = AStarPlanner(
            obstacle_manager=obstacle_manager,
            robot_config=robots_config['R1'],
            grid_size=55,
            vision_radius=5.0,
            k_att=1.0,
            k_rep=2.0,
            rep_radius=3.0
        )

class custom_algo:
    def __init__(self, robot_id , start , goal, obstacles: set):
        self.obstacles = obstacles
        self.global_path = self.a_star(start, goal, self.obstacles)

    def dist(self,a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def manhattan_dist(self,a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def is_valid_position(self, x, y, obstacles, grid_size, robot_size=1):
        for dx in range(robot_size):
            for dy in range(robot_size):
                px, py = x + dx, y + dy
                if not (0 <= px < grid_size and 0 <= py < grid_size):
                    return False
                if (px, py) in obstacles:
                    return False
        return True

    def get_neighbors(self, pos, obstacles, grid_size):
        x, y = pos
        neighbors = []

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                new_x, new_y = x + dx, y + dy

                if not self.is_valid_position(new_x, new_y, obstacles, grid_size):
                    continue

                if dx != 0 and dy != 0:
                    orth1 = (x + dx, y)
                    orth2 = (x, y + dy)

                    if (self.is_valid_position(orth1[0], orth1[1], obstacles, grid_size) and
                        self.is_valid_position(orth2[0], orth2[1], obstacles, grid_size)):
                        neighbors.append((new_x, new_y))
                else:
                    neighbors.append((new_x, new_y))

        return neighbors

    def a_star(self, start, goal, obstacles, grid_size=55):
        open_set = []
        heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.manhattan_dist(start, goal)}

        while open_set:
            current = heappop(open_set)[1]

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for neighbor in self.get_neighbors(current, obstacles, grid_size):
                tentative = g_score[current] + self.dist(current, neighbor)

                if neighbor not in g_score or tentative < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    f_score[neighbor] = tentative + self.manhattan_dist(neighbor, goal)
                    heappush(open_set, (f_score[neighbor], neighbor))

        return []
