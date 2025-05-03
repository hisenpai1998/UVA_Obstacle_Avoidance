import numpy as np
import time

class UAVNavigator:
    def __init__(self, sim_interface, keyboard_controller, 
                 depth_analyzer, visualizer, data_streamer,
                 obstacle_manager, apf_navigator, collision_detector, 
                 simulation_time, step_size):
        
        """Initialize the UAV navigator."""
        self.sim_interface          = sim_interface
        self.keyboard_controller    = keyboard_controller
        self.depth_analyzer         = depth_analyzer
        self.visualizer             = visualizer
        self.data_streamer          = data_streamer
        self.obstacle_manager       = obstacle_manager
        self.apf_navigator          = apf_navigator
        self.collision_detector     = collision_detector
        self.simulation_time        = simulation_time
        self.step_size              = step_size
        self.current_pos            = self.sim_interface.get_object_position('target')
        self.path_drawing           = None # Placeholder for path drawing object

    def start_simulation(self):
        """Start the simulation and initialize drawing objects."""
        self.sim_interface.start_simulation()
       
    def autonomous_move(self, apf_direction):
        """Move the UAV based on APF force in 3D space."""
        x, y, z = self.current_pos
        # Normalize the APF direction vector in 3D
        x += apf_direction[0] * self.step_size
        y += apf_direction[1] * self.step_size
        z += apf_direction[2] * self.step_size
        self.current_pos = [x, y, z]
        print(f"[DEBUG] Moving UAV to position: {self.current_pos}")

    def run(self):
        """Run the UAV simulation loop."""
        self.start_simulation() 
        last_step_time = time.time()
        try:
            while self.sim_interface.get_simulation_time() < self.simulation_time:
                if time.time() - last_step_time < 0.1:
                    continue
                last_step_time = time.time()

                # Step 1: Update dynamic obstacles
                self.obstacle_manager.update_obstacles(dt=0.1)

                # Step 2: Read depth data from the vision sensor
                depth_bytes, resolution     = self.sim_interface.get_vision_sensor_data()
                depth_resized               = self.depth_analyzer.process_depth_data(depth_bytes, resolution)

                # Step 3: Analyze 9 regions to determine safe regions and prioritize
                safe_regions, region_depths = self.depth_analyzer.analyze_9_regions(depth_resized)

                # Step 4: Choose the best direction based on prioritized regions
                label, best_angle           = self.depth_analyzer.choose_direction(safe_regions, region_depths, None, None)

                # Step 5: Inject virtual obstacles from depth using fixed offsets
                virtual_obs = []
                current_pos = self.sim_interface.get_object_position('target')
                offsets = {
                    'up-left':          [-0.5,  0.5, 0],
                    'up':               [   0,  0.5, 0],
                    'up-right':         [ 0.5,  0.5, 0],
                    'left':             [-0.5,    0, 0],
                    'front':            [   0,    0, 0],  # Center, might not need virtual obstacle
                    'right':            [ 0.5,    0, 0],
                    'down-left':        [-0.5, -0.5, 0],
                    'down':             [   0, -0.5, 0],
                    'down-right':       [ 0.5, -0.5, 0]
                }
                for region, avg in region_depths.items():
                    if avg < self.depth_analyzer.depth_threshold:
                        dx, dy, dz = offsets[region]
                        virtual_obs.append([current_pos[0] + dx, current_pos[1] + dy, current_pos[2] + dz])

                # Step 6: Compute APF force in 3D
                goal_pos        = self.sim_interface.get_object_position('goal')
                all_obstacles   = list(self.obstacle_manager.get_obstacle_positions().values()) + virtual_obs
                wall_positions  = [self.sim_interface.get_object_position(name) for name in self.sim_interface.object_paths['walls']]
                apf_direction   = self.apf_navigator.calculate_force(self.current_pos, goal_pos, all_obstacles, wall_positions)

                # Step 7: Check for manual control input
                manual_pos = self.keyboard_controller.update_position(self.current_pos, self.step_size)

                if manual_pos != self.current_pos:
                    self.current_pos = manual_pos
                else:
                    # If not Move UAV using APF direction
                    self.autonomous_move(apf_direction)
                
                self.sim_interface.set_object_position('target', self.current_pos)

                # Step 8: Visualize flight path
                uav_pos = self.sim_interface.get_object_position('drone')
                if self.path_drawing is not None:
                    # Add new path point z coordinate to the path drawing
                    self.sim_interface.sim.addDrawingObjectItem(self.path_drawing, uav_pos + [0, 1, 0])
                self.visualizer.update_depth_display(depth_resized)
                self.visualizer.update_path_display(uav_pos, goal_pos)

                # Don't uncomment this. They will give u bigger bugs. 
                # # Step 9: Check for collisions
                # if self.collision_detector.check_collision(uav_pos, all_obstacles):
                #     print("Collision detected! Stopping simulation.")
                #     continue

                # Step 10: Check distance to goal
                current_dist = np.linalg.norm(np.array(self.current_pos[:2]) - np.array(goal_pos[:2]))
                if current_dist < 1:
                    print("Goal reached!")
                    break

                # Step 11: Stream data
                frame_data = {
                    'depth':            depth_resized.copy(),
                    'label':            label,
                    'drone_position':   uav_pos,
                    'goal_position':    goal_pos,
                    'best_direction':   best_angle
                }
                self.data_streamer.stream_frame(frame_data)

                self.sim_interface.step()

        finally:
            self.cleanup()

    def cleanup(self):

        """Clean up resources."""

        self.keyboard_controller.stop()
        self.sim_interface.stop_simulation()
        self.data_streamer.close()
        self.visualizer.close()