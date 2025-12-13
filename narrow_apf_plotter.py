import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

from algo1 import custom_algo
from obstacle_creator import ObstacleMapManager
from random import choice


GRID = 55
interval_ms = 500  
max_limit_frames = 1000
initial , final = 0,54

# robot_configs = [
#     {"robot_id": "R_1", "start": (5, 39), "goal": (35, 27), "color": "red"},
#     {"robot_id": "R_2", "start": (49, 15), "goal": (19, 27), "color": "green"},
#     {"robot_id": "R_3", "start": (49, 32), "goal": (49, 15), "color": "blue"},                 Robot configurations for narrow path + Apf testing
#     {"robot_id": "R_4", "start": (5, 22), "goal": (5, 39), "color": "purple"},
    
# ]

robot_configs = [
    {"robot_id": "R_1", "start": (initial, initial), "goal": (final, final), "color": "red"},
    {"robot_id": "R_2", "start": (initial, final), "goal": (final, initial), "color": "green"},
    {"robot_id": "R_3", "start": (final, final), "goal": (initial, initial), "color": "blue"},                 
    {"robot_id": "R_4", "start": (final, initial), "goal": (initial, final), "color": "purple"},
]

# ----- OBSTACLE SETUP -----
obstacle_manager = ObstacleMapManager(
            grid_size=55,
            save_file="narrow_path.json"
        )

obstacles = set(obstacle_manager.get_obstacles())

def generate_robots(robot_configs):
    robots = {}
    Robot_details = {}
    priority_dict = {}
    actual_paths = {}

    for cfg in robot_configs:
        rid = cfg["robot_id"]
        start = cfg["start"]
        goal = cfg["goal"]
        color = cfg["color"]

        rob = custom_algo(robot_id=rid, start=start, goal=goal, color=color, obstacles = obstacles)
        robots[rid] = rob

        Robot_details[rid] = {
            "Start": rob.start,
            "Goal": rob.goal,
            "Robot_object": rob,
            "Robot_path": rob.global_path,
            "Robot_priority": rob.priority,
            "Narrow_Path_Status": rob.in_narrow_path,
            "BackTrack_status": False,
            "Reached_Goal": False,
            "actual_path_frame_counter": 0
        }

        priority_dict[rid] = rob.priority
        actual_paths[rid] = [rob.global_path[0]]
        Robot_details[rid]["Current_position"] = list(rob.global_path[0])

    return robots, Robot_details, priority_dict, actual_paths




robots, Robot_details, priority_dict, actual_paths = generate_robots(
    robot_configs
)


fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(0, GRID)
ax.set_ylim(0, GRID)
ax.set_xticks(range(GRID))
ax.set_yticks(range(GRID))
ax.grid(True)



# ----- DRAW OBSTACLES -----
for ox, oy in obstacles:
    ax.add_patch(Rectangle((ox, oy), 1, 1, color="black"))

# ----- ROBOTS -----

robot_rectangles = {}
robot_labels = {}
robot_trails = {}
trail_x = {}
trail_y = {}

for rid, details in Robot_details.items():
    x0, y0 = details["Robot_path"][0]

    rect = Rectangle((x0, y0), 1, 1, color=details["Robot_object"].color)
    ax.add_patch(rect)
    robot_rectangles[rid] = rect

    label = ax.text(x0, y0, rid.replace("R_", "R"), color="black", fontsize=8)
    robot_labels[rid] = label

    trail_x[rid] = [x0]
    trail_y[rid] = [y0]
    trail, = ax.plot(trail_x[rid], trail_y[rid], color=details["Robot_object"].color, linewidth=1.8)
    robot_trails[rid] = trail


pos_all = {rid: Robot_details[rid]["Current_position"] for rid in Robot_details}

priority_dict = {rid: Robot_details[rid]["Robot_priority"] for rid in Robot_details}

frame_counter = 0
last_processed_frame = -1



def check_adjacent(name, posA, other_positions):
    list_of_robots_in_avoidance_range = {} # store detected robots in vision range
    x, y = posA
    adj_offsets = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1)      ,     (0, 1),
        (1, -1),  (1, 0),  (1, 1)
    ]

    for dx, dy in adj_offsets: 
        for other_name, (ox, oy) in other_positions.items():
            if (x + dx, y + dy) == (ox, oy):
                list_of_robots_in_avoidance_range[other_name] = (ox, oy)
                break
    
        # print(f" {name} sees {list_of_robots_in_avoidance_range}")
    return list_of_robots_in_avoidance_range 

def check_visible(name, posA, other_positions):
    list_of_robots_in_visible_range = {} # store detected robots in vision range
    x, y = posA
    visible_offsets = [
        (i,j) for i in [-2, -1, 0, 1, 2] for j in [-2, -1, 0, 1, 2] if not (i == 0 and j == 0)
    ]
    for dx, dy in visible_offsets: 
        for other_name, (ox, oy) in other_positions.items():
            if (x + dx, y + dy) == (ox, oy):
                list_of_robots_in_visible_range[other_name] = (ox, oy)
                break
    return list_of_robots_in_visible_range



def update(frame):
    global last_processed_frame
    print(f"Frame: {frame}")
    print("*"*25)

    if frame <= last_processed_frame:
        return (list(robot_rectangles.values()) + list(robot_labels.values()) + list(robot_trails.values()))
    last_processed_frame = frame

    def algo_switch(name, current_pos, other_positions, robot_priority):
    
        goal = Robot_details[name]["Goal"]
        
        if (not other_positions) and (Robot_details[name]["Robot_object"].was_blocked):
            Robot_details[name]["Robot_object"].was_blocked = False
            new_path = Robot_details[name]["Robot_object"].a_star(current_pos, goal, obstacles)
            
            if new_path and len(new_path) > 1:
                Robot_details[name]["Robot_path"] = new_path
                Robot_details[name]["actual_path_frame_counter"] = 0
                next_pos = new_path[1]
                print(f"  {name}: IF branch - recomputed path, appending {next_pos}")
                actual_paths[name].append(next_pos)
                Robot_details[name]["actual_path_frame_counter"] += 1
                return next_pos
            else:
                print(f"  {name}: IF branch - path failed, staying at {current_pos}")
                actual_paths[name].append(current_pos)
                return current_pos
                
        elif not other_positions:
            planned_path = Robot_details[name]["Robot_path"]
            next_index = Robot_details[name]["actual_path_frame_counter"] + 1
            
            if next_index < len(planned_path):
                next_pos = planned_path[next_index]
                print(f"  {name}: ELIF branch - following A*, appending {next_pos}")
                actual_paths[name].append(next_pos)
                Robot_details[name]["actual_path_frame_counter"] += 1
                return next_pos
            else:
                print(f"  {name}: ELIF branch - end of path, staying at {current_pos}")
                actual_paths[name].append(current_pos)
                return current_pos
            
        elif not other_positions and Robot_details[name]["Narrow_Path_Status"]:
            planned_path = Robot_details[name]["Robot_path"]
            next_index = Robot_details[name]["actual_path_frame_counter"] + 1
            
            if next_index < len(planned_path):
                next_pos = planned_path[next_index]
                print(f"  {name}: NARROW PATH ELIF branch - following A*, appending {next_pos}")
                actual_paths[name].append(next_pos)
                Robot_details[name]["actual_path_frame_counter"] += 1
                return next_pos
            else:
                print(f"  {name}: NARROW PATH ELIF branch - end of path, staying at {current_pos}")
                actual_paths[name].append(current_pos)
                return current_pos
            
        elif other_positions and Robot_details[name]["Narrow_Path_Status"]:
            planned_path = Robot_details[name]["Robot_path"]
            next_index = Robot_details[name]["actual_path_frame_counter"] + 1

            for other_robo_name, pos in other_positions.items():
                if pos == planned_path[next_index]:
                    if robot_priority[name] > robot_priority[other_robo_name]:
                        next_pos = planned_path[next_index]
                        print(f"  {name}: NARROW PATH HIGH PRIORITY - following A*, appending {next_pos}")
                        actual_paths[name].append(next_pos)
                        Robot_details[name]["actual_path_frame_counter"] += 1
                        return next_pos
                    elif robot_priority[name] < robot_priority[other_robo_name]:
                        robots[name].BackTrack_status = True
                        next_pos = Robot_details[name]["Robot_object"].narrow_path_apf_choose_next(
                            current_pos, 
                            obstacles, 
                            other_positions, 
                            priority_dict = robot_priority
                        )
                        print(f"  {name}: NARROW PATH LOW PRIORITY - APF computed {next_pos} from {current_pos}")
                        actual_paths[name].append(next_pos)
                        Robot_details[name]["actual_path_frame_counter"] += 1
                        return next_pos
        
        elif other_positions:
            if robot_priority[name] > max([robot_priority[rn] for rn in other_positions.keys()]):
                planned_path = Robot_details[name]["Robot_path"]
                next_index = Robot_details[name]["actual_path_frame_counter"] + 1
                
                if next_index < len(planned_path):
                    next_pos = planned_path[next_index]
                    print(f"  {name}: HIGH PRIORITY - following A*, appending {next_pos}")
                    actual_paths[name].append(next_pos)
                    Robot_details[name]["actual_path_frame_counter"] += 1
                    return next_pos
                else:
                    print(f"  {name}: HIGH PRIORITY - end of path, staying at {current_pos}")
                    actual_paths[name].append(current_pos)
                    return current_pos
            else:
                Robot_details[name]["Robot_object"].was_blocked = True
                next_pos = Robot_details[name]["Robot_object"].simple_apf_choose_next(
                    current_pos, 
                    goal, 
                    obstacles, 
                    other_positions, 
                    priority_dict = robot_priority
                )
                print(f"  {name}: ELSE branch - APF computed {next_pos} from {current_pos}")
                actual_paths[name].append(next_pos)
                Robot_details[name]["actual_path_frame_counter"] += 1
                return next_pos

    def move_robot(name,path, current_pos, rect, label, trail_x, trail_y):        
        
        x, y = path[-1]
        current_pos = (x, y)

        rect.set_xy((x, y))
        label.set_position((x, y))

        if current_pos == Robot_details[name]["Goal"]:
            Robot_details[name]["Reached_Goal"] = True

        trail_x.append(x + 0.5)
        trail_y.append(y + 0.5)

        print(f"{name} moved to {current_pos}")
        pos_all[name] = current_pos
        return current_pos
    
    if frame > 0:
        for rid in Robot_details:
            Robot_details[rid]["Current_position"] = move_robot(
                rid,
                actual_paths[rid],
                Robot_details[rid]["Current_position"],
                robot_rectangles[rid],
                robot_labels[rid],
                trail_x[rid],
                trail_y[rid],
            )
            robot_trails[rid].set_data(trail_x[rid], trail_y[rid])




    # robots_in_visible_range_of_R_1 = check_visible(
    #     list(Robot_details.keys())[0],
    #     Robot_details[list(Robot_details.keys())[0]]["Current_position"], 
    #     {
    #         'R_2': Robot_details[list(Robot_details.keys())[1]]["Current_position"], 
    #         'R_3': Robot_details[list(Robot_details.keys())[2]]["Current_position"],
    #         'R_4': Robot_details[list(Robot_details.keys())[3]]["Current_position"],
    #         'R_5': Robot_details[list(Robot_details.keys())[4]]["Current_position"]
    #     }
    # )
    # robots_in_visible_range_of_R_2 = check_visible(
    #     list(Robot_details.keys())[1],
    #     Robot_details[list(Robot_details.keys())[1]]["Current_position"], 
    #     {
    #         'R_1': Robot_details[list(Robot_details.keys())[0]]["Current_position"], 
    #         'R_3': Robot_details[list(Robot_details.keys())[2]]["Current_position"],
    #         'R_4': Robot_details[list(Robot_details.keys())[3]]["Current_position"],
    #         'R_5': Robot_details[list(Robot_details.keys())[4]]["Current_position"]
    #     }
    # )

    # robots_in_visible_range_of_R_3 = check_visible(
    #     list(Robot_details.keys())[2],
    #     Robot_details[list(Robot_details.keys())[2]]["Current_position"], 
    #     {
    #         'R_1': Robot_details[list(Robot_details.keys())[0]]["Current_position"], 
    #         'R_2': Robot_details[list(Robot_details.keys())[1]]["Current_position"],
    #         'R_4': Robot_details[list(Robot_details.keys())[3]]["Current_position"],
    #         'R_5': Robot_details[list(Robot_details.keys())[4]]["Current_position"]
    #     }   
    # )

    # robots_in_visible_range_of_R_4 = check_visible(
    #     list(Robot_details.keys())[3],
    #     Robot_details[list(Robot_details.keys())[3]]["Current_position"], 
    #     {
    #         'R_1': Robot_details[list(Robot_details.keys())[0]]["Current_position"], 
    #         'R_2': Robot_details[list(Robot_details.keys())[1]]["Current_position"],
    #         'R_3': Robot_details[list(Robot_details.keys())[2]]["Current_position"],
    #         'R_5': Robot_details[list(Robot_details.keys())[4]]["Current_position"]
    #     }   
    # )

    robots_in_avoidance_range = {}

    for rid in Robot_details.keys():
        # Build dict of "all other robots"
        other_positions = {
            other_rid: Robot_details[other_rid]["Current_position"]
            for other_rid in Robot_details.keys()
            if other_rid != rid
        }

        robots_in_avoidance_range[rid] = check_adjacent(
            rid,
            Robot_details[rid]["Current_position"],
            other_positions
        )

    next_pos_all = {}

    for rid in Robot_details.keys():
        next_pos_all[rid] = algo_switch(
            rid,
            Robot_details[rid]["Current_position"],
            robots_in_avoidance_range[rid],
            priority_dict
        )


    info_dict = {name: [curr,next] for name, curr, next in zip(Robot_details.keys(),pos_all.values(), next_pos_all.values())}


    for name1 in list(Robot_details.keys()):
        for name2 in list(Robot_details.keys()):
            if name1 == name2:
                continue
            if actual_paths[name1][-1] == actual_paths[name2][-1]:
                if not Robot_details[name1]["Robot_object"].priority_resolution(name1,priority_dict[name1] , name2,priority_dict[name2]):
                    actual_paths[name1][-1] = actual_paths[name1][-2]

            if actual_paths[name1][-1] == actual_paths[name2][-2]:
                possible_neighbours = Robot_details[name2]["Robot_object"].get_neighbors_for_2nd_resolution(
                    actual_paths[name2][-2],obstacles, GRID, info_dict=info_dict
                )

                if possible_neighbours: 
                    actual_paths[name2][-1] = choice(possible_neighbours)

    for rid, rob in robots.items():
        if len(actual_paths[rid]) >= 2:
            rob.in_narrow_path = rob.narrow_path_detector(
                actual_paths[rid][-2],
                actual_paths[rid][-1]
            )

                 

    print("*"*25)

    return (list(robot_rectangles.values()) + list(robot_labels.values()) + list(robot_trails.values()))

ani = animation.FuncAnimation(
    fig,
    update,
    frames=max_limit_frames,
    interval=interval_ms,
    blit=False,
    repeat=False,
)

plt.show()