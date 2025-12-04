import math
import numpy as np
from heapq import heappush, heappop


DIRS = [
    (1, 0),   # 0°
    (1, 1),   # 45°
    (0, 1),   # 90°
    (-1, 1),  # 135°
    (-1, 0),  # 180°
    (-1, -1), # -135°
    (0, -1),  # -90°
    (1, -1)   # -45°
]

DIR_ANGLES = [0, 45, 90, 135, 180, -135, -90, -45]




class custom_algo:
    def __init__(self, robot_id , start , goal, obstacles: set):
        self.robot_id = robot_id
        self.obstacles = obstacles
        self.global_path = self.a_star(start, goal, self.obstacles)
        self.start = start
        self.goal = goal
        self.was_blocked = False
        self.reached_goal = False
        self.priority = 0
        self.set_priority(robot_id)

    def set_priority(self,robot_id):
        robo_no = int(robot_id.split('_')[1])
        self.priority = float(1/robo_no)

    def dist(self,a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def manhattan_dist(self,a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def is_valid_position_for_obstacle(self, x, y, obstacles, grid_size, robot_size=1 ,):
        for dx in range(robot_size):
            for dy in range(robot_size):
                px, py = x + dx, y + dy
                if not (0 <= px < grid_size and 0 <= py < grid_size):
                    return False
                if (px, py) in obstacles:
                    return False
        return True
    
    

    def get_neighbors_for_Astar(self, pos, obstacles, grid_size):
        x, y = pos
        neighbors = []

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                new_x, new_y = x + dx, y + dy

                if not self.is_valid_position_for_obstacle(new_x, new_y, obstacles, grid_size):
                    continue

                if dx != 0 and dy != 0:
                    orth1 = (x + dx, y)
                    orth2 = (x, y + dy)

                    if (self.is_valid_position_for_obstacle(orth1[0], orth1[1], obstacles, grid_size) and
                        self.is_valid_position_for_obstacle(orth2[0], orth2[1], obstacles, grid_size)):
                        neighbors.append((new_x, new_y))
                        
                else:
                    neighbors.append((new_x, new_y))

        return neighbors
    
    def is_valid_position_for_obstacle_APF(self, x, y, obstacles, grid_size, robot_size=1 ,):
        for dx in range(-2,3):
            for dy in range(-2,3):
                px, py = x + dx, y + dy
                if not (0 <= px < grid_size and 0 <= py < grid_size):
                    return False
                if (px, py) in obstacles:
                    return False
        return True
    
    def is_valid_position_for_robots(self, x, y, grid_size, robot_size=1, list_of_robots_in_avoidance_range : dict = {}):
        for dx in range(-1,2):
            for dy in range(-1,2):
                px, py = x + dx, y + dy
                if not (0 <= px < grid_size and 0 <= py < grid_size):
                    return False
                for robot_pos in list_of_robots_in_avoidance_range.values():
                    if (px, py) == robot_pos:
                        return False
        return True
    
    def get_neighbors_for_APF(self, pos, obstacles, grid_size, list_of_robots_in_avoidance_range : dict = {}):
        x, y = pos
        neighbors = []

        for dx in range(-1,2):
            for dy in range(-1,2):
                if dx == 0 and dy == 0:
                    continue

                new_x, new_y = x + dx, y + dy

                if not self.is_valid_position_for_obstacle(new_x, new_y, obstacles, grid_size):
                    continue

                if not self.is_valid_position_for_robots(new_x, new_y, grid_size, list_of_robots_in_avoidance_range=list_of_robots_in_avoidance_range):
                    continue 
        
                else:
                    neighbors.append((new_x, new_y))

        return neighbors
    

    def a_star(self, start, goal, obstacles, grid_size=55):
        start = tuple(start)
        goal = tuple(goal)

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

            for neighbor in self.get_neighbors_for_Astar(current, obstacles, grid_size):
                tentative = g_score[current] + self.dist(current, neighbor)

                if neighbor not in g_score or tentative < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    f_score[neighbor] = tentative + self.manhattan_dist(neighbor, goal)
                    heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def simple_apf_choose_next(self, pos, goal, obstacles, list_of_robots_in_avoidance_range : dict = {}, priority_dict : dict = {}):
        """
        pos: (x, y) current robot position
        goal: (x, y) goal position
        obstacles: list of (x, y) obstacle positions
        neighbours: list of (x, y) available neighbor positions

        returns: (x, y) selected neighbour

        """
        neighbours = self.get_neighbors_for_APF(pos, obstacles, grid_size=55 ,list_of_robots_in_avoidance_range=list_of_robots_in_avoidance_range)

        

        if neighbours == []:
            return pos  # no available move so stay in place

        pos = np.array(pos, dtype=float)
        goal = np.array(goal, dtype=float)

        # Attractive force: simple linear pull
        F_att = (goal - pos)

        # Repulsive force: very basic: sum of inverse distance squared
        F_rep = np.zeros(2, dtype=float)
        rep_radius = 4.0
        k_rep = 50.0

        higher_robots = {}

        for ox, oy in obstacles:
            obs = np.array([ox, oy], dtype=float)
            dvec = pos - obs
            dist = np.linalg.norm(dvec)
            if dist < 1e-6:
                continue
            if dist <= rep_radius:
                F_rep += k_rep * (1.0 / (dist**2)) * (dvec / dist)

        for rname,rpos in list_of_robots_in_avoidance_range.items():
            if rname == self.robot_id:
                continue
            if priority_dict[rname] < self.priority:
                continue
            higher_robots[rname] = list_of_robots_in_avoidance_range[rname]

        for rname,rpos in higher_robots.items():
            rx, ry = rpos
            rob = np.array([rx, ry], dtype=float)
            dvec = pos - rob
            dist = np.linalg.norm(dvec)
            if dist < 1e-6:
                continue
            if dist <= rep_radius:
                F_rep += k_rep * (1.0 / (dist**2)) * (dvec / dist)


        # Total force
        F = F_att + F_rep

        # Angle of resultant force
        angle = math.degrees(math.atan2(F[1], F[0]))

        # Find nearest 45° direction
        best_idx = min(range(8), key=lambda i: abs(angle - DIR_ANGLES[i]))
        chosen_dir = DIRS[best_idx]

        # Convert selected direction to actual neighbor
        dx, dy = chosen_dir
        final_pos = (int(pos[0] + dx), int(pos[1] + dy))

        # If that neighbor is not available, fallback to nearest available one
        if final_pos not in neighbours:
            # Choose from neighbours: which one’s direction is closest?
            def angle_to(npos):
                vec = np.array(npos) - pos
                return math.degrees(math.atan2(vec[1], vec[0]))

            best_nb = min(neighbours, key=lambda n: abs(angle - angle_to(n)))
            return best_nb

        return final_pos
    
    def priority_resolution(self, name, p1 , other_name, p2):

        if p1 > p2:
            return True
        else:
             return False
        