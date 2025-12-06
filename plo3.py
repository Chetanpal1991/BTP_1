import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

from algo1 import custom_algo
from obstacle_creator import ObstacleMapManager


# ----- OBSTACLE SETUP -----
obstacle_manager = ObstacleMapManager(
            grid_size=55,
            save_file="yielding_robot_obstacle_maps.json"
        )

obstacles = set(obstacle_manager.get_obstacles())

initial , final = 13, 35

# ----- ROBOT SETUP -----
Robo1 = custom_algo(robot_id='R_1', start=(initial, initial), goal=(final, final), obstacles=obstacles)
Robo2 = custom_algo(robot_id='R_2', start=(final, final), goal=(initial, initial), obstacles=obstacles)
Robo3 = custom_algo(robot_id='R_3', start=(initial, final), goal=(final, initial), obstacles=obstacles)

Robot_details = {
    Robo1.robot_id: {
        "Start": Robo1.start,
        "Goal": Robo1.goal,
        "Robot_object": Robo1,
        "Robot_path": Robo1.global_path,
        "Robot_priority": Robo1.priority,
        "Reached_Goal": False,
        "actual_path_frame_counter": 0

    },
    Robo2.robot_id: {
        "Start": Robo2.start,
        "Goal": Robo2.goal,
        "Robot_object": Robo2,
        "Robot_path": Robo2.global_path,
        "Robot_priority": Robo2.priority,
        "Reached_Goal": False,
        "actual_path_frame_counter": 0
    },
    Robo3.robot_id: {
        "Start": Robo3.start,
        "Goal": Robo3.goal,
        "Robot_object": Robo3,
        "Robot_path": Robo3.global_path,
        "Robot_priority": Robo3.priority,
        "Reached_Goal": False,
        "actual_path_frame_counter": 0
    }
}

# object_dict = {
#     'R_1': Robo1,
#     'R_2': Robo2,
#     'R_3': Robo3
# }

priority_dict = {
    'R_1': Robo1.priority,
    'R_2': Robo2.priority,
    'R_3': Robo3.priority
}


# path1 = Robo1.global_path
# path2 = Robo2.global_path
# path3 = Robo3.global_path

# robot_paths = {Robo1.robot_id: path1, Robo2.robot_id: path2, Robo3.robot_id: path3}

GRID = 55
interval_ms = 500  
max_limit_frames = 1000

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


fig, ax = plt.subplots(figsize=(10, 25))
ax.set_xlim(0, GRID)
ax.set_ylim(0, GRID)
ax.set_xticks(range(GRID))
ax.set_yticks(range(GRID))
ax.grid(True)

actual_paths = {
    Robo1.robot_id: [Robo1.global_path[0]],
    Robo2.robot_id: [Robo2.global_path[0]],
    Robo3.robot_id: [Robo3.global_path[0]]
}

# ----- DRAW OBSTACLES -----
for ox, oy in obstacles:
    ax.add_patch(Rectangle((ox, oy), 1, 1, color="black"))

# ----- ROBOTS -----
r1 = Rectangle((Robot_details[Robo1.robot_id]["Robot_path"][0][0], Robot_details[Robo1.robot_id]["Robot_path"][0][1]), 1, 1, color="red")
r2 = Rectangle((Robot_details[Robo2.robot_id]["Robot_path"][0][0], Robot_details[Robo2.robot_id]["Robot_path"][0][1]), 1, 1, color="blue")
r3 = Rectangle((Robot_details[Robo3.robot_id]["Robot_path"][0][0], Robot_details[Robo3.robot_id]["Robot_path"][0][1]), 1, 1, color="green")
ax.add_patch(r1)
ax.add_patch(r2)
ax.add_patch(r3)
# ----- ROBOT LABELS -----
label1 = ax.text(Robot_details[Robo1.robot_id]["Robot_path"][0][0] - 0.5, Robot_details[Robo1.robot_id]["Robot_path"][0][1] - 0.5, "R1", color="Black", fontsize=8)
label2 = ax.text(Robot_details[Robo2.robot_id]["Robot_path"][0][0] + 0.5, Robot_details[Robo2.robot_id]["Robot_path"][0][1] + 0.5, "R2", color="Black", fontsize=8)
label3 = ax.text(Robot_details[Robo3.robot_id]["Robot_path"][0][0] + 0.5, Robot_details[Robo3.robot_id]["Robot_path"][0][1] + 0.5, "R3", color="Black", fontsize=8)

# ----- TRAILS -----
trail1_x, trail1_y = [Robot_details[Robo1.robot_id]["Robot_path"][0][0]], [Robot_details[Robo1.robot_id]["Robot_path"][0][1]]
trail2_x, trail2_y = [Robot_details[Robo2.robot_id]["Robot_path"][0][0]], [Robot_details[Robo2.robot_id]["Robot_path"][0][1]]
trail3_x, trail3_y = [Robot_details[Robo3.robot_id]["Robot_path"][0][0]], [Robot_details[Robo3.robot_id]["Robot_path"][0][1]]

trail1, = ax.plot(trail1_x, trail1_y, "r-", linewidth=1.8)
trail2, = ax.plot(trail2_x, trail2_y, "b-", linewidth=1.8)
trail3, = ax.plot(trail3_x, trail3_y, "g-", linewidth=1.8)



current_r1 = list(Robot_details[Robo1.robot_id]["Robot_path"][0])
current_r2 = list(Robot_details[Robo2.robot_id]["Robot_path"][0])
current_r3 = list(Robot_details[Robo3.robot_id]["Robot_path"][0])

Robot_details[Robo1.robot_id]["Current_position"] = current_r1
Robot_details[Robo2.robot_id]["Current_position"] = current_r2
Robot_details[Robo3.robot_id]["Current_position"] = current_r3

pos_all = {
    Robo1.robot_id: Robot_details[Robo1.robot_id]["Current_position"],
    Robo2.robot_id: Robot_details[Robo2.robot_id]["Current_position"],
    Robo3.robot_id: Robot_details[Robo3.robot_id]["Current_position"]
}

priority_dict = {
    Robo1.robot_id: Robot_details[Robo1.robot_id]["Robot_priority"],
    Robo2.robot_id: Robot_details[Robo2.robot_id]["Robot_priority"],
    Robo3.robot_id: Robot_details[Robo3.robot_id]["Robot_priority"]
}

frame_counter = 0
last_processed_frame = -1

def update(frame):
    global current_r1, current_r2, current_r3,last_processed_frame
    print(f"Frame: {frame}")
    print("*"*25)

    if frame <= last_processed_frame:
        return r1, r2, r3, label1, label2, label3, trail1, trail2, trail3
    
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
            else:
                print(f"  {name}: IF branch - path failed, staying at {current_pos}")
                actual_paths[name].append(current_pos)
                
        elif not other_positions:
            planned_path = Robot_details[name]["Robot_path"]
            next_index = Robot_details[name]["actual_path_frame_counter"] + 1
            
            if next_index < len(planned_path):
                next_pos = planned_path[next_index]
                print(f"  {name}: ELIF branch - following A*, appending {next_pos}")
                actual_paths[name].append(next_pos)
                Robot_details[name]["actual_path_frame_counter"] += 1
            else:
                print(f"  {name}: ELIF branch - end of path, staying at {current_pos}")
                actual_paths[name].append(current_pos)
        
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
        return current_pos
    
    # def robot_func(name):
    #     if frame >0:
    #         Robot_details[name]["Current_position"] = move_robot(                           #Move robot to new position
    #             name,
    #             actual_paths[name], 
    #             Robot_details[name]["Current_position"], 
    #             {'R_1': r1, 'R_2': r2, 'R_3': r3}[name], 
    #             {'R_1': label1, 'R_2': label2, 'R_3': label3}[name], 
    #             {'R_1': trail1_x, 'R_2': trail2_x, 'R_3': trail3_x}[name], 
    #             {'R_1': trail1_y, 'R_2': trail2_y, 'R_3': trail3_y}[name],
    #         )
    #         {'R_1': trail1, 'R_2': trail2, 'R_3': trail3}[name].set_data(               #Update trail
    #             {'R_1': trail1_x, 'R_2': trail2_x, 'R_3': trail3_x}[name], 
    #             {'R_1': trail1_y, 'R_2': trail2_y, 'R_3': trail3_y}[name],
    #         )

    #         robots_in_visible_range = check_visible(
    #             name,
    #             Robot_details[name]["Current_position"], 
    #             {k: v for k, v in pos_all.items() if k != name}
    #         )

    #         robots_in_avoidance_range = check_adjacent(
    #             name,
    #             Robot_details[name]["Current_position"], 
    #             {k: v for k, v in pos_all.items() if k != name}
    #         )
    #         algo_switch(
    #             name,
    #             Robot_details[name]["Current_position"], 
    #             robots_in_avoidance_range,
    #             priority_dict
    #         )
    
    if frame > 0:
        Robot_details[Robo1.robot_id]["Current_position"] = move_robot(
            Robo1.robot_id,
            actual_paths[Robo1.robot_id], 
            Robot_details[Robo1.robot_id]["Current_position"], 
            r1, label1, trail1_x, trail1_y,
            # Robot_details[Robo1.robot_id]["actual_path_frame_counter"]
        )
        Robot_details[Robo2.robot_id]["Current_position"] = move_robot(
            Robo2.robot_id,
            actual_paths[Robo2.robot_id], 
            Robot_details[Robo2.robot_id]["Current_position"], 
            r2, label2, trail2_x, trail2_y,
            # Robot_details[Robo2.robot_id]["actual_path_frame_counter"]
        )
        Robot_details[Robo3.robot_id]["Current_position"] = move_robot(
            Robo3.robot_id,
            actual_paths[Robo3.robot_id], 
            Robot_details[Robo3.robot_id]["Current_position"], 
            r3, label3, trail3_x, trail3_y,
            # Robot_details[Robo3.robot_id]["actual_path_frame_counter"]
        )

        trail1.set_data(trail1_x, trail1_y)
        trail2.set_data(trail2_x, trail2_y)
        trail3.set_data(trail3_x, trail3_y)


    # Now compute NEXT positions for all robots
    robots_in_visible_range_of_R_1 = check_visible(
        list(Robot_details.keys())[0], 
        Robot_details[list(Robot_details.keys())[0]]["Current_position"], 
        {
            'R_2': Robot_details[list(Robot_details.keys())[1]]["Current_position"], 
            'R_3': Robot_details[list(Robot_details.keys())[2]]["Current_position"]
        }
    )
    robots_in_visible_range_of_R_2 = check_visible(
        list(Robot_details.keys())[1], 
        Robot_details[list(Robot_details.keys())[1]]["Current_position"], 
        {
            'R_1': Robot_details[list(Robot_details.keys())[0]]["Current_position"], 
            'R_3': Robot_details[list(Robot_details.keys())[2]]["Current_position"]
        }
    )
    robots_in_visible_range_of_R_3 = check_visible(
        list(Robot_details.keys())[2], 
        Robot_details[list(Robot_details.keys())[2]]["Current_position"], 
        {
            'R_1': Robot_details[list(Robot_details.keys())[0]]["Current_position"], 
            'R_2': Robot_details[list(Robot_details.keys())[1]]["Current_position"]
        }
    )

    robots_in_avoidance_range_of_R_1 = check_adjacent(
        list(Robot_details.keys())[0],
        Robot_details[list(Robot_details.keys())[0]]["Current_position"], 
        {
            'R_2': Robot_details[list(Robot_details.keys())[1]]["Current_position"], 
            'R_3': Robot_details[list(Robot_details.keys())[2]]["Current_position"]
        }
    )

    robots_in_avoidance_range_of_R_2 = check_adjacent(
        list(Robot_details.keys())[1],
        Robot_details[list(Robot_details.keys())[1]]["Current_position"], 
        {
            'R_1': Robot_details[list(Robot_details.keys())[0]]["Current_position"], 
            'R_3': Robot_details[list(Robot_details.keys())[2]]["Current_position"]
        }
    )

    robots_in_avoidance_range_of_R_3 = check_adjacent(
        list(Robot_details.keys())[2],
        Robot_details[list(Robot_details.keys())[2]]["Current_position"], 
        {
            'R_1': Robot_details[list(Robot_details.keys())[0]]["Current_position"], 
            'R_2': Robot_details[list(Robot_details.keys())[1]]["Current_position"]
        }
    )

    # Compute next positions and append to actual_paths
    algo_switch(
        Robo1.robot_id, 
        Robot_details[Robo1.robot_id]["Current_position"], 
        robots_in_avoidance_range_of_R_1, 
        priority_dict
    )
    algo_switch(
        Robo2.robot_id, 
        Robot_details[Robo2.robot_id]["Current_position"], 
        robots_in_avoidance_range_of_R_2, 
        priority_dict
    )
    algo_switch(
        Robo3.robot_id, 
        Robot_details[Robo3.robot_id]["Current_position"], 

        robots_in_avoidance_range_of_R_3, 
        priority_dict
    )

    # robot_func(Robo1.robot_id)
    # robot_func(Robo2.robot_id)
    # robot_func(Robo3.robot_id)

    for name1 in list(Robot_details.keys()):
        for name2 in list(Robot_details.keys()):
            if name1 == name2:
                continue
            if actual_paths[name1][-1] == actual_paths[name2][-1]:
                if not Robot_details[name1]["Robot_object"].priority_resolution(name1,priority_dict[name1] , name2,priority_dict[name2]):
                    actual_paths[name1][-1] = actual_paths[name1][-2]

                 

    print("*"*25)

    return r1, r2, r3, label1, label2, label3, trail1, trail2, trail3


ani = animation.FuncAnimation(
    fig,
    update,
    frames=max_limit_frames,
    interval=interval_ms,
    blit=False,
    repeat=False,
)

plt.show()
