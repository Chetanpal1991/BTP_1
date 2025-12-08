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

# ----- ROBOT SETUP -----
Robo1 = custom_algo(robot_id='R_1', start=(17, 15), goal=(45, 45), obstacles=obstacles)
Robo2 = custom_algo(robot_id='R_2', start=(12, 42), goal=(42, 12), obstacles=obstacles)
Robo3 = custom_algo(robot_id='R_3', start=(17, 42), goal=(42, 17), obstacles=obstacles)

Robot_details = {
    Robo1.robot_id: {
        "Start": Robo1.start,
        "Goal": Robo1.goal,
        "Robot_object": Robo1,
        "Robot_path": Robo1.global_path,
        "Robot_priority": Robo1.priority,

    },
    Robo2.robot_id: {
        "Start": Robo2.start,
        "Goal": Robo2.goal,
        "Robot_object": Robo2,
        "Robot_path": Robo2.global_path,
        "Robot_priority": Robo2.priority
    },
    Robo3.robot_id: {
        "Start": Robo3.start,
        "Goal": Robo3.goal,
        "Robot_object": Robo3,
        "Robot_path": Robo3.global_path,
        "Robot_priority": Robo3.priority
    }
}

# object_dict = {
#     'R_1': Robo1,
#     'R_2': Robo2,
#     'R_3': Robo3
# }

# priority_dict = {
#     'R_1': Robo1.priority,
#     'R_2': Robo2.priority,
#     'R_3': Robo3.priority
# }


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



def update(frame):
    global current_r1, current_r2, current_r3

    def priority_yield_check(robot_id, Robots_in_visible_range: dict):
        if not Robots_in_visible_range:
            return   # No robots in visible range, this robot can move
        for other_robot_id, current_pos in Robots_in_visible_range.items():
            robot_next_pos = Robot_details[robot_id]["Robot_path"][frame+1] if frame +1 < len(Robot_details[robot_id]["Robot_path"]) else Robot_details[robot_id]["Robot_path"][-1]
            other_robot_next_pos = Robot_details[other_robot_id]["Robot_path"][frame+1] if frame +1 < len(Robot_details[other_robot_id]["Robot_path"]) else Robot_details[other_robot_id]["Robot_path"][-1] 
            if robot_next_pos == other_robot_next_pos:
                higher_priority_robot_id = Robot_details[robot_id]["Robot_object"].priority_resolution( robot_id,Robot_details[robot_id]["Robot_priority"], other_robot_id, Robot_details[other_robot_id]["Robot_priority"])    
                if higher_priority_robot_id == robot_id: 
                    return   # This robot has higher priority, can move
                else:
                    # Lower priority robot, must stay in place
                    # Adjust the path to stay in current position
                    Robot_details[robot_id]["Robot_path"].insert(frame + 1, Robot_details[robot_id]["Robot_path"][frame])
                    return 
            else:
                continue

    # -------- HELPER: MOVE ROBOT SAFELY --------
    def move_robot(path, current_pos, rect, label, trail_x, trail_y):
        if frame >= len(path):
            return current_pos  # robot already done

        x, y = path[frame]
        current_pos = (x, y)

        rect.set_xy((x, y))
        label.set_position((x, y))

        trail_x.append(x + 0.5)
        trail_y.append(y + 0.5)
        return current_pos
    
    # for Robot_id in Robot_details.keys():
    #     for other_robot_id in Robot_details.keys():
    #         if Robot_id == other_robot_id:
    #             continue
    #         if Robot_details[Robot_id]["Robot_path"][frame] == Robot_details[other_robot_id]["Robot_path"][frame]:
    #             higher_priority_number = max(Robot_details[Robot_id]["Robot_priority"], Robot_details[other_robot_id]["Robot_priority"])
    #             if Robot_details[Robot_id]["Robot_priority"] == higher_priority_number:
    #                 continue
    #             else:
    #                 # Lower priority robot, must stay in place
    #                 # Adjust the path to stay in current position
    #                 Robot_details[Robot_id]["Robot_path"][frame] = Robot_details[Robot_id]["Robot_path"][frame-1]
    
    Robot_details[Robo1.robot_id]["Current_position"] = move_robot(Robot_details[Robo1.robot_id]["Robot_path"], Robot_details[Robo1.robot_id]["Current_position"], r1, label1, trail1_x, trail1_y)
    Robot_details[Robo2.robot_id]["Current_position"] = move_robot(Robot_details[Robo2.robot_id]["Robot_path"], Robot_details[Robo2.robot_id]["Current_position"], r2, label2, trail2_x, trail2_y)
    Robot_details[Robo3.robot_id]["Current_position"] = move_robot(Robot_details[Robo3.robot_id]["Robot_path"], Robot_details[Robo3.robot_id]["Current_position"], r3, label3, trail3_x, trail3_y)

    trail1.set_data(trail1_x, trail1_y)
    trail2.set_data(trail2_x, trail2_y)
    trail3.set_data(trail3_x, trail3_y)

    robots_in_visible_range_of_R_1 = check_visible(list(Robot_details.keys())[0], Robot_details[list(Robot_details.keys())[0]]["Current_position"], {'R_2': Robot_details[list(Robot_details.keys())[1]]["Current_position"], 'R_3': Robot_details[list(Robot_details.keys())[2]]["Current_position"]})
    robots_in_visible_range_of_R_2 = check_visible(list(Robot_details.keys())[1], Robot_details[list(Robot_details.keys())[1]]["Current_position"], {'R_1': Robot_details[list(Robot_details.keys())[0]]["Current_position"], 'R_3': Robot_details[list(Robot_details.keys())[2]]["Current_position"]})
    robots_in_visible_range_of_R_3 = check_visible(list(Robot_details.keys())[2], Robot_details[list(Robot_details.keys())[2]]["Current_position"], {'R_1': Robot_details[list(Robot_details.keys())[0]]["Current_position"], 'R_2': Robot_details[list(Robot_details.keys())[1]]["Current_position"]})

    priority_yield_check(list(Robot_details.keys())[0], robots_in_visible_range_of_R_1)
    priority_yield_check(list(Robot_details.keys())[1], robots_in_visible_range_of_R_2)
    priority_yield_check(list(Robot_details.keys())[2], robots_in_visible_range_of_R_3)

    robots_in_apf_range_of_R_1 = check_adjacent(list(Robot_details.keys())[0], Robot_details[list(Robot_details.keys())[0]]["Current_position"], {'R_2': Robot_details[list(Robot_details.keys())[1]]["Current_position"], 'R_3': Robot_details[list(Robot_details.keys())[2]]["Current_position"]})
    robots_in_apf_range_of_R_2 = check_adjacent(list(Robot_details.keys())[1], Robot_details[list(Robot_details.keys())[1]]["Current_position"], {'R_1': Robot_details[list(Robot_details.keys())[0]]["Current_position"], 'R_3': Robot_details[list(Robot_details.keys())[2]]["Current_position"]})
    robots_in_apf_range_of_R_3 = check_adjacent(list(Robot_details.keys())[2], Robot_details[list(Robot_details.keys())[2]]["Current_position"], {'R_1': Robot_details[list(Robot_details.keys())[0]]["Current_position"], 'R_2': Robot_details[list(Robot_details.keys())[1]]["Current_position"]})

    if robots_in_apf_range_of_R_1:
        Robot_details[Robo1.robot_id]["Robot_object"].was_blocked = True
        next_pos_R_1 = Robot_details[Robo1.robot_id]["Robot_object"].simple_apf_choose_next(
            Robot_details[Robo1.robot_id]["Current_position"],
            Robot_details[Robo1.robot_id]["Goal"],
            obstacles,
            list_of_robots_in_avoidance_range=robots_in_apf_range_of_R_1,
            priority_dict={rid: d["Robot_priority"] for rid, d in Robot_details.items()}
        )
        Robot_details[Robo1.robot_id]["Robot_path"][frame+1] = next_pos_R_1

    elif Robot_details[Robo1.robot_id]["Robot_object"].was_blocked:
        # Recalculate global path using A*
        new_path = Robot_details[Robo1.robot_id]["Robot_object"].a_star(
            Robot_details[Robo1.robot_id]["Current_position"],
            Robot_details[Robo1.robot_id]["Goal"],
            obstacles
        )
        if new_path:
            Robot_details[Robo1.robot_id]["Robot_path"] = Robot_details[Robo1.robot_id]["Robot_path"][:frame+1] + new_path[1:]  # Update path from current position
        Robot_details[Robo1.robot_id]["Robot_object"].was_blocked = False

    if robots_in_apf_range_of_R_2:
        Robot_details[Robo2.robot_id]["Robot_object"].was_blocked = True
        next_pos_R_2 = Robot_details[Robo2.robot_id]["Robot_object"].simple_apf_choose_next(
            Robot_details[Robo2.robot_id]["Current_position"],
            Robot_details[Robo2.robot_id]["Goal"],
            obstacles,
            list_of_robots_in_avoidance_range=robots_in_apf_range_of_R_2,
            priority_dict={rid: d["Robot_priority"] for rid, d in Robot_details.items()}
        )
        Robot_details[Robo2.robot_id]["Robot_path"][frame+1] = next_pos_R_2

    elif Robot_details[Robo2.robot_id]["Robot_object"].was_blocked:
        # Recalculate global path using A*
        new_path = Robot_details[Robo2.robot_id]["Robot_object"].a_star(
            Robot_details[Robo2.robot_id]["Current_position"],
            Robot_details[Robo2.robot_id]["Goal"],
            obstacles
        )
        if new_path:
            Robot_details[Robo2.robot_id]["Robot_path"] = Robot_details[Robo2.robot_id]["Robot_path"][:frame+1] + new_path[1:]  # Update path from current position
        Robot_details[Robo2.robot_id]["Robot_object"].was_blocked = False

    if robots_in_apf_range_of_R_3:
        Robot_details[Robo3.robot_id]["Robot_object"].was_blocked = True
        next_pos_R_3 = Robot_details[Robo3.robot_id]["Robot_object"].simple_apf_choose_next(
            Robot_details[Robo3.robot_id]["Current_position"],
            Robot_details[Robo3.robot_id]["Goal"],
            obstacles,
            list_of_robots_in_avoidance_range=robots_in_apf_range_of_R_3,
        priority_dict={rid: d["Robot_priority"] for rid, d in Robot_details.items()}
        )
        Robot_details[Robo3.robot_id]["Robot_path"][frame+1] = next_pos_R_3

    elif Robot_details[Robo3.robot_id]["Robot_object"].was_blocked:
        # Recalculate global path using A*
        new_path = Robot_details[Robo3.robot_id]["Robot_object"].a_star(
            Robot_details[Robo3.robot_id]["Current_position"],
            Robot_details[Robo3.robot_id]["Goal"],
            obstacles
        )
        if new_path:
            Robot_details[Robo3.robot_id]["Robot_path"] = Robot_details[Robo3.robot_id]["Robot_path"][:frame+1] + new_path[1:]  # Update path from current position
        Robot_details[Robo3.robot_id]["Robot_object"].was_blocked = False


    # Update the paths based on APF decisions)

    

    
    # print(f"R1 sees: {robots_in_visible_range_of_R_1}")
    # print(f"R2 sees: {robots_in_visible_range_of_R_2}")
    # print(f"R3 sees: {robots_in_visible_range_of_R_3}")

    return r1, r2, r3, label1, label2, label3, trail1, trail2, trail3


ani = animation.FuncAnimation(
    fig,
    update,
    frames=max_limit_frames,
    interval=interval_ms,
    blit=True,
    repeat=False,
)

plt.show()
