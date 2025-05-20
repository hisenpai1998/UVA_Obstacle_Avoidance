import numpy    as np
import time

from config     import DEPTH_THRESHOLD

class UAVNavigator:
    def __init__(self, sim_interface, keyboard_controller, 
                 depth_analyzer, visualizer, data_streamer,
                 obstacle_manager, apf_navigator, collision_detector, 
                 simulation_time, step_size):
        """
        Initialize the UAV navigator.
        """
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
        self.path_drawing           = None                                                  # Placeholder for path drawing object

    def start_simulation(self):
        """
        Start the simulation and initialize drawing objects.
        """
        self.sim_interface.start_simulation()
       

    def autonomous_move(self, apf_direction, direction_vector, region_depths):
        """
        Move the UAV based on APF force and depth-based vector in 3D space.
        """
        x, y, z = self.current_pos
        apf_direction = np.array(apf_direction)
        direction_vector = np.array(direction_vector)

        if region_depths.get('front', float('inf')) < DEPTH_THRESHOLD * 2:
            move_vector = direction_vector * self.step_size
        else:
            move_vector = apf_direction * self.step_size

        x += move_vector[0]
        y += move_vector[1]
        z += move_vector[2]
        self.current_pos = [x, y, z]

        print(f"[DEBUG] Moving UAV to position: {self.current_pos}")
        
        return move_vector

    def run(self):
        """
        Run the UAV simulation loop.
        """
        self.start_simulation()
        last_step_time = time.time()
        try:
            while self.sim_interface.get_simulation_time() < self.simulation_time:
                if time.time() - last_step_time < 0.2: # Save 5 frame per second instead for 10
                    continue
                last_step_time = time.time()

                # Step 1: Update dynamic obstacles
                self.obstacle_manager.update_obstacles(dt=0.1)

                # Step 2: Read depth data from the vision sensor
                depth_bytes, resolution = self.sim_interface.get_vision_sensor_data()
                depth_resized = self.depth_analyzer.process_depth_data(depth_bytes, resolution)

                goal_pos = self.sim_interface.get_object_position('goal')
                
                # Step 3: Get UAV's current yaw
                _, _, uav_yaw = self.sim_interface.sim.getObjectOrientation(
                    self.sim_interface.handles['drone'], -1)

                # Step 4: Handle target orientation (yaw) based on keyboard input
                _, _, current_target_yaw = self.sim_interface.sim.getObjectOrientation(
                    self.sim_interface.handles['target'], -1)
                new_target_yaw = self.keyboard_controller.update_yaw(current_target_yaw)
                self.sim_interface.sim.setObjectOrientation(
                    self.sim_interface.handles['target'], -1, [0, 0, new_target_yaw])

                # Step 5: Update target position based on keyboard input relative to UAV's yaw
                old_pos = self.current_pos.copy()
                new_pos = self.keyboard_controller.update_position_with_yaw(old_pos, uav_yaw, self.step_size)
                self.current_pos = new_pos
                self.sim_interface.set_object_position('target', new_pos)
                move_vector = np.array(new_pos) - np.array(old_pos)

                # Step 6: Determine label based on movement vector or rotation
                label_num, label_str = 6, 'stop'  # Default to stop
                if np.linalg.norm(move_vector) > 0:
                    direction = move_vector / np.linalg.norm(move_vector)
                    label_num, label_str, _ = self.depth_analyzer.vector_to_label(direction, uav_yaw)
                elif self.keyboard_controller.keys_pressed['z']:
                    label_num, label_str = 7, 'rotate_left'
                elif self.keyboard_controller.keys_pressed['x']:
                    label_num, label_str = 8, 'rotate_right'

                # Step 7: Visualize flight path
                uav_pos = self.sim_interface.get_object_position('drone')
                if self.path_drawing is not None:
                    self.sim_interface.sim.addDrawingObjectItem(self.path_drawing, uav_pos + [0, 1, 0])
                self.visualizer.update_depth_display(depth_resized)
                self.visualizer.update_path_display(uav_pos, goal_pos)

                # Step 8: Check distance to goal
                current_dist = np.linalg.norm(np.array(self.current_pos[:2]) - np.array(goal_pos[:2]))
                if current_dist < 1:
                    print("Goal reached!")
                    break

                # Step 9: Stream data
                frame_data = {
                    'depth':            depth_resized.copy(),
                    'label':            label_num,
                    'label_str':        label_str,
                    'drone_position':   uav_pos,
                    'goal_position':    goal_pos,
                    'move_vector':      move_vector.tolist()  # Store actual move vector
                }
                self.data_streamer.stream_frame(frame_data)
                self.sim_interface.step()

        finally:
            self.cleanup()

    def cleanup(self):
        """
        Clean up resources.
        """
        self.keyboard_controller.stop()
        self.sim_interface.stop_simulation()
        self.data_streamer.close()
        self.visualizer.close()