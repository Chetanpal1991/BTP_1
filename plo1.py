import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

from algo1 import custom_algo
from obstacle_creator import ObstacleMapManager

obstacle_manager = ObstacleMapManager(
            grid_size=55,
            save_file="yielding_robot_obstacle_maps.json"
        )

obstacles = set(obstacle_manager.get_obstacles())

Robo1 = custom_algo(robot_id='R_1', start=(20, 20), goal=(40, 40), obstacles=obstacles)
Robo2 = custom_algo(robot_id='R_2', start=(40, 40), goal=(20, 20), obstacles=obstacles)
Robo3 = custom_algo(robot_id='R_3', start=(20, 40), goal=(40, 20), obstacles=obstacles)

priority_dict = {
    'R_1': Robo1.priority,
    'R_2': Robo2.priority,
    'R_3': Robo3.priority
}


path1 = Robo1.global_path
path2 = Robo2.global_path
path3 = Robo3.global_path

GRID = 55
interval_ms = 500  
max_limit_frames = 1000

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
r1 = Rectangle((path1[0][0], path1[0][1]), 1, 1, color="red")
r2 = Rectangle((path2[0][0], path2[0][1]), 1, 1, color="blue")
r3 = Rectangle((path3[0][0], path3[0][1]), 1, 1, color="green")
ax.add_patch(r1)
ax.add_patch(r2)
ax.add_patch(r3)
# ----- ROBOT LABELS -----
label1 = ax.text(path1[0][0] + 0.5, path1[0][1] + 0.5, "R1", color="white", fontsize=8)
label2 = ax.text(path2[0][0] + 0.5, path2[0][1] + 0.5, "R2", color="white", fontsize=8)
label3 = ax.text(path3[0][0] + 0.5, path3[0][1] + 0.5, "R3", color="white", fontsize=8)

# ----- TRAILS -----
trail1_x, trail1_y = [path1[0][0]], [path1[0][1]]
trail2_x, trail2_y = [path2[0][0]], [path2[0][1]]
trail3_x, trail3_y = [path3[0][0]], [path3[0][1]]

trail1, = ax.plot(trail1_x, trail1_y, "r-", linewidth=1.8)
trail2, = ax.plot(trail2_x, trail2_y, "b-", linewidth=1.8)
trail3, = ax.plot(trail3_x, trail3_y, "g-", linewidth=1.8)



current_r1 = list(path1[0])
current_r2 = list(path2[0])
current_r3 = list(path3[0])



def update(frame):
    global current_r1, current_r2, current_r3

    # -------- HELPER: MOVE ROBOT SAFELY --------
    def move_robot(path, current_pos, rect, label, trail_x, trail_y):
        if frame >= len(path):
            return current_pos  # robot already done

        x, y = path[frame]
        current_pos = (x, y)

        rect.set_xy((x, y))
        label.set_position((x + 0.5, y + 0.5))

        trail_x.append(x + 0.5)
        trail_y.append(y + 0.5)
        return current_pos

    # ----------- PHASE 1: UPDATE POSITIONS -------------
    current_r1 = move_robot(path1, current_r1, r1, label1, trail1_x, trail1_y)
    current_r2 = move_robot(path2, current_r2, r2, label2, trail2_x, trail2_y)
    current_r3 = move_robot(path3, current_r3, r3, label3, trail3_x, trail3_y)

    trail1.set_data(trail1_x, trail1_y)
    trail2.set_data(trail2_x, trail2_y)
    trail3.set_data(trail3_x, trail3_y)

    # ---------- PHASE 2: HANDLING EACH ROBOT ----------

    def external_range(name,robot: custom_algo,current_and_next_pos_dict):
        visible_robots = check_visible(
            name,
            current_and_next_pos_dict[name][0],
            {
                other_name: positions[0]
                for other_name, positions in current_and_next_pos_dict.items()
                if other_name != name
            }
        )
        if visible_robots:
            for other_name, other_pos in visible_robots.items():
                current_robot_next_pos = current_and_next_pos_dict[name][1]
                colloiding_robot_next_pos = current_and_next_pos_dict[other_name][1]
                if current_robot_next_pos == colloiding_robot_next_pos:
                    print(f"Collision detected between {name} and {other_name} at position {current_robot_next_pos}")
                    higher_name, its_next_pos =robot.priority_resolution(name, other_name, current_and_next_pos_dict,priority_dict)
                    return higher_name, its_next_pos

        else:
            return name, current_and_next_pos_dict[name][1]

                    


                

    def process_robot(name, robot, path:list, current_pos, others):

        avoidable = check_adjacent(name, current_pos, others)
        global next_pos

        if avoidable:
            robot.was_blocked = True
            next_pos = robot.simple_apf_choose_next(
                current_pos, robot.goal, obstacles,
                list_of_robots_in_avoidance_range=avoidable,
                priority_dict=priority_dict
            )
            Insert next APF step
            if frame + 1 < len(path):
                path.insert(frame + 1, next_pos)    
            else:
                path.append(next_pos)

        else:
            if robot.was_blocked:
                robot.was_blocked = False
                # Recompute A*
                new_path = robot.a_star(current_pos, robot.goal, obstacles)
                # Replace future steps
                if len(new_path) > 1:
                    path[frame+1:] = new_path[1:]

        return next_pos

    # -------------- PROCESS ALL THREE -----------------
    next_pos_R1 = process_robot(Robo1.robot_id, Robo1, path1, current_r1, {
        Robo2.robot_id: current_r2, Robo3.robot_id: current_r3
    })

    next_pos_R2 = process_robot(Robo2.robot_id, Robo2, path2, current_r2, {
        Robo1.robot_id: current_r1, Robo3.robot_id: current_r3
    })

    next_pos_R3 = process_robot(Robo3.robot_id, Robo3, path3, current_r3, {
        Robo1.robot_id: current_r1, Robo2.robot_id: current_r2
    })

    current_and_next_pos_dict = {
        Robo1.robot_id: [current_r1, next_pos_R1],
        Robo2.robot_id: [current_r2, next_pos_R2],
        Robo3.robot_id: [current_r3, next_pos_R3]
    }

    r1_higher_robot_name, r1_its_final_next_pos =external_range(Robo1.robot_id, Robo1, current_and_next_pos_dict)
    r2_higher_robot_name, r2_its_final_next_pos =external_range(Robo2.robot_id, Robo2, current_and_next_pos_dict)    
    r3_higher_robot_name, r3_its_final_next_pos =external_range(Robo3.robot_id, Robo3, current_and_next_pos_dict)  


    for higher_name in [r1_higher_robot_name, r2_higher_robot_name, r3_higher_robot_name]:
        higher_name_next_pos = current_and_next_pos_dict[higher_name][1]
        if higher_name == 'R_1':
            if frame + 1 < len(path1):
                path1.insert(frame + 1, higher_name_next_pos)    
            else:
                path1.append(higher_name_next_pos)
        elif higher_name == 'R_2':
            if frame + 1 < len(path2):
                path2.insert(frame + 1, higher_name_next_pos)    
            else:
                path2.append(higher_name_next_pos)

        elif higher_name == 'R_3':
            if frame + 1 < len(path3):
                path3.insert(frame + 1, higher_name_next_pos)    
            else:
                path3.append(higher_name_next_pos)
    

    return r1, r2, r3, label1, label2, label3, trail1, trail2, trail3

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
    if list_of_robots_in_avoidance_range:
        print(f" {name} sees {list_of_robots_in_avoidance_range}")
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



ani = animation.FuncAnimation(
    fig,
    update,
    frames=max_limit_frames,
    interval=interval_ms,
    blit=True,
    repeat=False,
)

plt.show()
