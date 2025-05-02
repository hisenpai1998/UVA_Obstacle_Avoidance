import numpy            as np

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

    def autonomous_move(self, label, apf_direction):

        """Move the UAV based on safe direction and APF force."""

        x, y, z = self.current_pos
        # Use safe direction if available
        if label == "forward":
            y += self.step_size
        elif label == "right":
            x += self.step_size
        elif label == "left":
            x -= self.step_size
        elif label == "down":
            z -= self.step_size
        elif label == "up":
            z += self.step_size

        # Adjust movement with APF direction
        if label != "stop":
            apf_x, apf_y = apf_direction * self.step_size
            x += apf_x
            y += apf_y

        self.current_pos = [x, y, z]

    def run(self):

        """Run the UAV simulation loop."""

        try:
            while self.sim_interface.get_simulation_time() < self.simulation_time:

                # Step 1: Update dynamic obstacles
                self.obstacle_manager.update_obstacles(dt=0.1)


                # Step 2: Read depth map
                depth_bytes, resolution  = self.sim_interface.get_vision_sensor_data()
                depth_resized            = self.depth_analyzer.process_depth_data(
                                                depth_bytes, 
                                                resolution
                                            )

                # Step 3: Analyze 9 regions to determine safe regions and prioritize
                safe_regions, region_depths = self.depth_analyzer.analyze_9_regions(depth_resized)  

                # Step 4: Choose the best direction based on prioritized regions
                label, best_angle = self.depth_analyzer.choose_direction(safe_regions, region_depths, None, None)
                
                # Step 5: Update and draw visualization
                self.visualizer.update_depth_display(depth_resized)

                # Step 5: Virtual obstacle from depth (if needed)
                virtual_obstacles = []
                current_pos = self.sim_interface.get_object_position('target')
                for region, avg in region_depths.items():
                    if avg < self.depth_analyzer.depth_threshold:
                        angle = self.depth_analyzer.map_region_to_angle(region)
                        x = current_pos[0] + 0.5 * np.cos(angle)
                        y = current_pos[1] + 0.5 * np.sin(angle)
                        virtual_obstacles.append([x, y, current_pos[2]])
                self.obstacle_manager.add_virtual_obstacles(virtual_obstacles)

                # Step 6: Calculate APF force
                uav_pos     = self.sim_interface.get_object_position('drone')
                goal_pos    = self.sim_interface.get_object_position('goal')
                obstacle_positions  = self.obstacle_manager.get_obstacle_positions()
                apf_direction       = self.apf_navigator.calculate_force(uav_pos, goal_pos, obstacle_positions)

                # Step 7: Move the UAV
                self.current_pos = self.keyboard_controller.update_position(self.current_pos, self.step_size)  # Manual control
                self.autonomous_move(label, apf_direction)
                self.sim_interface.set_object_position('target', self.current_pos)

                # Step 7: Visualize flight path
                self.visualizer.update_path_display(uav_pos, goal_pos)

                # Step 8: Check for collisions
                if self.collision_detector.check_collision(uav_pos, obstacle_positions):
                    print("Collision detected! Stopping simulation.")
                    break

                
                # Step 9: Check distance to goal
                target_pos      = self.sim_interface.get_object_position('target')
                goal_pos        = self.sim_interface.get_object_position('goal')
                current_dist    = np.linalg.norm(np.array(target_pos[:2]) - np.array(goal_pos[:2]))
                if current_dist < 1:
                    print("Goal reached!")
                    break

                # Step 10: Stream data
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