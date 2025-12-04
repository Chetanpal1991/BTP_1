import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import numpy as np

from obstacle_creator import ObstacleMapManager

# Grid configuration
GRID_SIZE = 55
ROBOT_SIZE = 1
MIN_ROBOT_DISTANCE = 3  # Vision radius for APF mode detection

class MultiRobotPlotter:
    def __init__(self, grid_size=55):
        self.grid_size = grid_size
        self.obstacle_manager = ObstacleMapManager(
            grid_size=grid_size,
            save_file="yielding_robot_obstacle_maps.json"
        )

        self.robots_config = {
            'R1': {'start': (0, 0), 'goal': (54, 54)},
            'R2': {'start': (0, 54), 'goal': (54, 0)},
            # 'R3': {'start': (54, 0), 'goal': (0, 54)},
            # 'R4': {'start': (54, 54), 'goal': (0, 0)}
        }

        for config in self.robots_config.values():
            self.obstacle_manager.add_protected_position(*config['start'])
            self.obstacle_manager.add_protected_position(*config['goal'])

        self.algorithm = None
        self.algorithm_class = None
        self.algorithm_params = {}
        self.algorithms = {}

        self.colors = {'R1': 'red', 'R2': 'blue', 'R3': 'green', 'R4': 'orange'}

        self.animation_running = False
        self.ani = None
        self.traces = {name: [] for name in self.robots_config}
        self.current_positions = {}
        self.frame_count = 0
        self.max_frames = 1000
        
        # Track APF mode per robot
        self.apf_mode_active = {name: False for name in self.robots_config}

        self._setup_plot()

        if len(self.obstacle_manager.get_obstacles()) == 0:
            self.obstacle_manager.create_complex_maze()

        self._draw_setup()

    def set_algorithm(self, algorithm_class, **algorithm_params):
        self.algorithm_class = algorithm_class
        self.algorithm_params = algorithm_params
        self.algorithms = {}
        print(f"\n{'='*70}")
        print(f"Algorithm set to: {algorithm_class.__name__}")
        if algorithm_params:
            print(f"Parameters: {algorithm_params}")
        print(f"{'='*70}\n")

    def _setup_plot(self):
        self.fig, self.ax = plt.subplots(figsize=(14, 10))
        button_height = 0.04
        button_width = 0.08
        button_spacing = 0.09
        start_ax = plt.axes([0.02, 0.02, button_width, button_height])
        self.start_button = Button(start_ax, 'Start')
        self.start_button.on_clicked(self._on_start_button)
        clear_ax = plt.axes([0.02 + button_spacing, 0.02, button_width, button_height])
        self.clear_button = Button(clear_ax, 'Clear All')
        self.clear_button.on_clicked(self._on_clear_button)
        simple_ax = plt.axes([0.02 + 2*button_spacing, 0.02, button_width, button_height])
        self.simple_button = Button(simple_ax, 'Simple')
        self.simple_button.on_clicked(self._on_simple_maze)
        complex_ax = plt.axes([0.02 + 3*button_spacing, 0.02, button_width, button_height])
        self.complex_button = Button(complex_ax, 'Complex')
        self.complex_button.on_clicked(self._on_complex_maze)
        cross_ax = plt.axes([0.02 + 4*button_spacing, 0.02, button_width, button_height])
        self.cross_button = Button(cross_ax, 'Cross')
        self.cross_button.on_clicked(self._on_cross_pattern)
        diamond_ax = plt.axes([0.02 + 5*button_spacing, 0.02, button_width, button_height])
        self.diamond_button = Button(diamond_ax, 'Diamond')
        self.diamond_button.on_clicked(self._on_diamond_pattern)
        zigzag_ax = plt.axes([0.02 + 6*button_spacing, 0.02, button_width, button_height])
        self.zigzag_button = Button(zigzag_ax, 'Zigzag')
        self.zigzag_button.on_clicked(self._on_zigzag_pattern)
        self.fig.canvas.mpl_connect('button_press_event', self._on_mouse_click)
        self._add_instructions()

    def _add_instructions(self):
        plt.figtext(0.02, 0.94, 'Multi-Robot Path Planning - Hybrid A* + APF',
                   fontsize=14, weight='bold', color='darkblue')
        plt.figtext(0.02, 0.91, 'Intelligent Navigation:', fontsize=12, weight='bold')
        plt.figtext(0.02, 0.88, '✓ A* for global path planning', fontsize=10, color='green')
        plt.figtext(0.02, 0.85, '✓ APF for local collision avoidance', fontsize=10, color='green')
        plt.figtext(0.02, 0.82, '• Click grid cells to add/remove obstacles', fontsize=10)
        plt.figtext(0.02, 0.79, '• Use buttons to load preset patterns', fontsize=10)
        plt.figtext(0.02, 0.76, f'• Vision radius: {MIN_ROBOT_DISTANCE} cells', fontsize=10, weight='bold')
        file_info = self.obstacle_manager.get_save_file_info()
        if file_info['exists']:
            plt.figtext(0.02, 0.72, f"💾 Maps auto-saved: {file_info['modified']}",
                       fontsize=9, color='green')
        else:
            plt.figtext(0.02, 0.72, f"💾 Maps will auto-save on first change",
                       fontsize=9, color='orange')
        plt.tight_layout()
        plt.subplots_adjust(bottom=0.12, left=0.22)

    def _draw_setup(self):
        self.ax.clear()
        self._setup_axes()
        current_mode = self.obstacle_manager.get_current_map_type()
        obstacle_count = len(self.obstacle_manager.get_obstacles())
        algorithm_name = self.algorithm_class.__name__ if self.algorithm_class else "None"
        self.ax.set_title(f'Multi-Robot Path Planning - {current_mode.title()} Mode\n'
                          f'Algorithm: {algorithm_name} | Obstacles: {obstacle_count}\n'
                          f'Click to add/remove obstacles, then press Start')
        self._draw_obstacles()
        self._draw_robots_and_goals()
        plt.draw()

    def _setup_axes(self):
        self.ax.set_xticks(np.arange(0, self.grid_size + 1, 5))
        self.ax.set_yticks(np.arange(0, self.grid_size + 1, 5))
        self.ax.grid(True, which='major', color='lightgray', linestyle='-', alpha=0.5, linewidth=0.5)
        self.ax.set_xlim(0, self.grid_size)
        self.ax.set_ylim(0, self.grid_size)
        self.ax.set_aspect('equal')

    def _draw_obstacles(self):
        obstacles = self.obstacle_manager.get_obstacles()
        for obs in obstacles:
            patch = plt.Rectangle((obs[0], obs[1]), 1, 1, color='black', alpha=0.8)
            self.ax.add_patch(patch)

    def _draw_robots_and_goals(self):
        for name, config in self.robots_config.items():
            start_x, start_y = config['start']
            self.ax.add_patch(plt.Rectangle((start_x, start_y), ROBOT_SIZE, ROBOT_SIZE,
                                           color=self.colors[name], alpha=0.9))
            self.ax.text(start_x + ROBOT_SIZE/2, start_y + ROBOT_SIZE/2, name,
                         fontsize=8, color='white', ha='center', va='center', weight='bold')
            goal_x, goal_y = config['goal']
            self.ax.add_patch(plt.Rectangle((goal_x, goal_y), ROBOT_SIZE, ROBOT_SIZE,
                                           color=self.colors[name], alpha=0.3,
                                           linestyle='--', fill=False, linewidth=2))
            self.ax.text(goal_x + ROBOT_SIZE/2, goal_y + ROBOT_SIZE/2, f'{name}_G',
                         fontsize=8, ha='center', va='center', alpha=0.7)
        for pos in self.obstacle_manager.get_protected_positions():
            self.ax.add_patch(plt.Rectangle((pos[0], pos[1]), 1, 1, color='red', alpha=0.2))

    def _update_animation_frame(self, frame):
        self.ax.clear()
        self._setup_axes()
        algorithm_name = self.algorithm_class.__name__ if self.algorithm_class else "Unknown"
        
        # Count robots in APF mode
        apf_count = sum(1 for active in self.apf_mode_active.values() if active)
        
        self.ax.set_title(f'Multi-Robot Path Planning | Algorithm: {algorithm_name}\n'
                          f'Frame {self.frame_count} | Time: {self.frame_count * 0.25:.2f}s | '
                          f'APF Mode: {apf_count}/4 robots')
        self._draw_obstacles()

        for name, config in self.robots_config.items():
            goal_x, goal_y = config['goal']
            self.ax.add_patch(plt.Rectangle((goal_x, goal_y), ROBOT_SIZE, ROBOT_SIZE,
                                           color=self.colors[name], alpha=0.3,
                                           linestyle='--', fill=False, linewidth=2))
            self.ax.text(goal_x + ROBOT_SIZE/2, goal_y + ROBOT_SIZE/2, f'{name}_G',
                         fontsize=8, ha='center', va='center', alpha=0.7)

        try:
            if self.algorithms:
                next_positions = {}
                for name in sorted(self.robots_config.keys()):
                    algo = self.algorithms.get(name)
                    if algo is None:
                        next_positions[name] = self.current_positions.get(name, self.robots_config[name]['start'])
                        self.apf_mode_active[name] = False
                        continue

                    # Build single-robot input
                    single_input = {}
                    single_input['R'] = self.current_positions.get(name, self.robots_config[name]['start'])
                    
                    # Include all other robots as occupancy info
                    for other_name, pos in self.current_positions.items():
                        if other_name != name:
                            single_input[other_name] = pos

                    # Compute peers inside vision radius
                    cx, cy = single_input['R']
                    vision_radius = MIN_ROBOT_DISTANCE
                    peers_in_vision = []
                    
                    for other_name, pos in self.current_positions.items():
                        if other_name == name:
                            continue
                        ox, oy = pos
                        # Calculate Euclidean distance
                        distance = ((ox - cx)**2 + (oy - cy)**2)**0.5
                        if distance <= vision_radius:
                            peers_in_vision.append((ox, oy))
                    
                    single_input['peers'] = peers_in_vision
                    
                    # Track APF mode status
                    self.apf_mode_active[name] = len(peers_in_vision) > 0

                    # Call the planner for this robot
                    try:
                        result = algo.update_positions(single_input)
                        if isinstance(result, dict):
                            if 'R' in result:
                                next_positions[name] = result['R']
                            elif name in result:
                                next_positions[name] = result[name]
                            else:
                                next_positions[name] = self.current_positions.get(name, self.robots_config[name]['start'])
                        else:
                            next_positions[name] = self.current_positions.get(name, self.robots_config[name]['start'])
                    except Exception as e:
                        print(f"ERROR: algorithm for {name} failed at frame {self.frame_count}: {e}")
                        next_positions[name] = self.current_positions.get(name, self.robots_config[name]['start'])
                        self.apf_mode_active[name] = False

                # Commit next positions
                self.current_positions = next_positions
            else:
                if self.algorithm is not None:
                    self.current_positions = self.algorithm.update_positions(self.current_positions)
                else:
                    pass

        except Exception as e:
            print(f"\n{'='*70}")
            print(f"ERROR: Algorithm failed at frame {self.frame_count}")
            print(f"Exception: {e}")
            print(f"{'='*70}\n")
            self._stop_animation()
            return

        # Draw robots at current positions with traces
        for name, pos in self.current_positions.items():
            x, y = pos
            center = (x + ROBOT_SIZE / 2, y + ROBOT_SIZE / 2)
            self.traces[name].append(center)
            if len(self.traces[name]) > 1:
                trace_x, trace_y = zip(*self.traces[name])
                self.ax.plot(trace_x, trace_y, color=self.colors[name],
                             linewidth=2, alpha=0.7)
            
            # Draw robot with different appearance based on mode
            robot_alpha = 0.9 if not self.apf_mode_active[name] else 1.0
            self.ax.add_patch(plt.Rectangle((x, y), ROBOT_SIZE, ROBOT_SIZE,
                                           color=self.colors[name], alpha=robot_alpha,
                                           linewidth=2 if self.apf_mode_active[name] else 1,
                                           edgecolor='yellow' if self.apf_mode_active[name] else None))
            
            # Draw vision field circle (more accurate representation)
            # vision_circle = plt.Circle((x + ROBOT_SIZE/2, y + ROBOT_SIZE/2),
            #                           MIN_ROBOT_DISTANCE,
            #                           color=self.colors[name], fill=False,
            #                           alpha=0.3 if self.apf_mode_active[name] else 0.15,
            #                           linestyle='--', linewidth=2 if self.apf_mode_active[name] else 1)
            # self.ax.add_patch(vision_circle)

            vision_square = plt.Rectangle((x - ROBOT_SIZE, y - ROBOT_SIZE),
                                   MIN_ROBOT_DISTANCE+1,
                                   MIN_ROBOT_DISTANCE+1,
                                   color=self.colors[name], fill=False,
                                   alpha=0.15, linestyle='--', linewidth=1)
            self.ax.add_patch(vision_square)
            
            # Label robot with mode indicator
            mode_indicator = " [APF]" if self.apf_mode_active[name] else ""
            self.ax.text(x + ROBOT_SIZE / 2, y + ROBOT_SIZE / 2, name + mode_indicator,
                         fontsize=7, color='white', ha='center', va='center', weight='bold')

        self.frame_count += 1

        if self._should_stop_animation():
            self._on_animation_complete()

    def _should_stop_animation(self):
        if self.frame_count >= self.max_frames:
            print(f"\nReached max frames ({self.max_frames}), stopping.")
            return True
        all_at_goal = True
        for name, pos in self.current_positions.items():
            goal = self.robots_config[name]['goal']
            if (int(pos[0]), int(pos[1])) != goal:
                all_at_goal = False
                break
        if all_at_goal:
            print(f"\nAll robots reached goals at frame {self.frame_count}")
            return True
        if len(self.traces[list(self.robots_config.keys())[0]]) > 20:
            all_stationary = True
            for name in self.robots_config.keys():
                recent_positions = self.traces[name][-10:]
                if len(set(recent_positions)) > 1:
                    all_stationary = False
                    break
            if all_stationary:
                print(f"\nRobots stationary for 10 frames, stopping at frame {self.frame_count}")
                return True
        return False

    def _start_animation(self):
        if self.algorithm_class is None:
            print("\n" + "="*70)
            print("ERROR: No algorithm set!")
            print("Use plotter.set_algorithm(AlgorithmClass, **params) first")
            print("="*70 + "\n")
            return

        print("\n" + "="*70)
        print("STARTING NEW SIMULATION")
        print("="*70)

        self.start_button.label.set_text("Planning...")
        plt.draw()

        self.traces = {name: [] for name in self.robots_config}
        self.frame_count = 0
        self.apf_mode_active = {name: False for name in self.robots_config}

        try:
            print("\nInitializing per-robot algorithm instances...")
            self.algorithms = {}
            for name, cfg in self.robots_config.items():
                print(f"  - Initializing for {name}")
                # Pass APF parameters from algorithm_params
                algo = self.algorithm_class(
                    self.obstacle_manager, 
                    cfg, 
                    grid_size=self.grid_size,
                    vision_radius=MIN_ROBOT_DISTANCE,
                    **self.algorithm_params
                )
                self.algorithms[name] = algo
        except Exception as e:
            print(f"\nERROR: Failed to initialize algorithms: {e}")
            self.start_button.label.set_text("Start")
            return

        try:
            print("\nAsking algorithms to compute A* paths...")
            paths_count = 0
            for name, algo in self.algorithms.items():
                print(f"  Computing path for {name}...")
                result = algo.compute_all_paths()
                if isinstance(result, dict):
                    path = None
                    if 'R' in result:
                        path = result['R']
                    elif name in result:
                        path = result[name]
                    else:
                        vals = list(result.values())
                        if vals:
                            path = vals[0]
                    if path:
                        paths_count += 1
                        print(f"    ✓ Path length: {len(path)} steps")
                else:
                    pass

            if paths_count == 0:
                print("ERROR: Algorithms returned no paths!")
                self.start_button.label.set_text("Start")
                return

            print(f"✓ Algorithms computed paths for {paths_count} robots")
        except Exception as e:
            print(f"\nERROR: Algorithms failed to compute paths: {e}")
            self.start_button.label.set_text("Start")
            return

        self.current_positions = {name: tuple(map(float, config['start']))
                                  for name, config in self.robots_config.items()}

        print(f"\n" + "-"*70)
        print("STARTING ANIMATION")
        print(f"Vision Radius: {MIN_ROBOT_DISTANCE} cells")
        print("Mode: Hybrid A* (global) + APF (local collision avoidance)")
        print("-"*70)

        self.animation_running = True
        self.start_button.label.set_text("Stop")

        self.ani = animation.FuncAnimation(
            self.fig,
            self._update_animation_frame,
            interval=250,
            repeat=False,
            cache_frame_data=False
        )

        plt.draw()

    def _stop_animation(self):
        if self.ani is not None:
            self.ani.event_source.stop()
            self.ani = None

        self.animation_running = False
        self.start_button.label.set_text("Start")
        self.traces = {name: [] for name in self.robots_config}
        self.current_positions = {}
        self.frame_count = 0
        self.apf_mode_active = {name: False for name in self.robots_config}
        print("\n" + "="*70)
        print("SIMULATION STOPPED")
        print("="*70)
        self._draw_setup()

    def _on_animation_complete(self):
        self.animation_running = False
        self.start_button.label.set_text("Start")
        print("\n" + "="*70)
        print("SIMULATION COMPLETED")
        print("="*70)
        if self.algorithms:
            all_reached = True
            for name, pos in self.current_positions.items():
                goal = self.robots_config[name]['goal']
                pos_int = (int(pos[0]), int(pos[1]))
                if pos_int != goal:
                    all_reached = False
                    print(f"  {name}: Did not reach goal (at {pos_int}, goal {goal})")
                else:
                    print(f"  {name}: ✓ Reached goal {goal}")
            if all_reached:
                print("\n✓ All robots successfully reached their goals!")
            else:
                print("\n✗ Some robots did not reach their goals")
        print(f"\nTotal frames: {self.frame_count}")
        print(f"Total time: {self.frame_count * 0.25:.2f}s")
        print("="*70)
        plt.draw()

    def _on_start_button(self, event):
        if self.animation_running:
            self._stop_animation()
        else:
            self._start_animation()

    def _on_clear_button(self, event):
        if self.animation_running:
            self._stop_animation()
        self.obstacle_manager.clear_all_obstacles()
        self._draw_setup()

    def _on_simple_maze(self, event):
        if self.animation_running:
            self._stop_animation()
        self.obstacle_manager.create_simple_maze()
        self._draw_setup()

    def _on_complex_maze(self, event):
        if self.animation_running:
            self._stop_animation()
        self.obstacle_manager.create_complex_maze()
        self._draw_setup()

    def _on_cross_pattern(self, event):
        if self.animation_running:
            self._stop_animation()
        self.obstacle_manager.create_custom_pattern("cross")
        self._draw_setup()

    def _on_diamond_pattern(self, event):
        if self.animation_running:
            self._stop_animation()
        self.obstacle_manager.create_custom_pattern("diamond")
        self._draw_setup()

    def _on_zigzag_pattern(self, event):
        if self.animation_running:
            self._stop_animation()
        self.obstacle_manager.create_custom_pattern("zigzag")
        self._draw_setup()

    def _on_mouse_click(self, event):
        if self.animation_running:
            return
        if event.inaxes != self.ax:
            return
        x = int(event.xdata)
        y = int(event.ydata)
        if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
            self.obstacle_manager.toggle_obstacle(x, y)
            self._draw_setup()

    def show(self):
        plt.show()

def main():
    print("="*70)
    print("Multi-Robot Path Planning Simulator - Hybrid A* + APF")
    print("="*70)
    print("Initializing...")
    plotter = MultiRobotPlotter(grid_size=55)
    from algo import AStarPlanner
    plotter.set_algorithm(
        AStarPlanner,
        k_att=1.0,      # Attractive force strength
        k_rep=2.0,      # Repulsive force strength
        rep_radius=3.0  # Repulsion activation distance
    )
    print("✓ Plotter initialized")
    print("✓ Hybrid A* + APF algorithm configured")
    print(f"✓ Vision radius: {MIN_ROBOT_DISTANCE} cells")
    print("✓ Ready to start simulation")
    print("\nInstructions:")
    print("  1. Click on grid to add/remove obstacles")
    print("  2. Use buttons to load preset patterns")
    print("  3. Press 'Start' to begin simulation")
    print("\nBehavior:")
    print("  • A* computes global path for each robot")
    print("  • Robot follows A* path when no peers nearby")
    print("  • Switches to APF when peers enter vision radius")
    print("  • APF uses 8-directional movement (N, NE, E, SE, S, SW, W, NW)")
    print("  • Returns to A* path when collision risk clears")
    print("  • Yellow border indicates APF mode active")
    print("="*70)
    plotter.show()

if __name__ == "__main__":
    main()