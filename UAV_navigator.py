import numpy as np
import time

from config import DEPTH_THRESHOLD

class UAVNavigator:
    def __init__(self, sim_interface, keyboard_controller, 
                 depth_analyzer, visualizer, data_streamer,
                 obstacle_manager, apf_navigator, collision_detector, 
                 simulation_time, step_size):
        """
        Initialize the UAV navigator.
        """
        self.sim_interface              = sim_interface
        self.keyboard_controller        = keyboard_controller
        self.depth_analyzer             = depth_analyzer
        self.visualizer                 = visualizer
        self.data_streamer              = data_streamer
        self.obstacle_manager           = obstacle_manager
        self.apf_navigator              = apf_navigator
        self.collision_detector         = collision_detector
        self.simulation_time            = simulation_time
        self.step_size                  = step_size
        self.path_drawing               = None                            # Placeholder for path drawing object
        
        self.current_label              = 6                               # Default to stop
        self.current_label_str          = 'stop'
        
        self.last_movement_time         = time.time()
        self.movement_timeout           = 0.5                             # Time in seconds before reverting to stop state
        
        self.last_uav_pos               = None                            # Will be initialized in first frame
        self.last_uav_yaw               = None                            # Will be initialized in first frame
        self.position_threshold         = 0.015                           # Maximum position change per axis for stop detection
        self.yaw_threshold              = 0.01                            # Maximum yaw change to consider as rotation
        self.stop_debug                 = True                            # Enable debug output for stop detection
        self.consecutive_stops          = 0                               # Counter for consecutive stable positions
        self.required_stops             = 3                               # Number of consecutive stable positions needed for stop
        self.current_pos                = self.sim_interface.get_object_position('target')


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

    def update_obstacles(self, dt=0.1):
        """
        Update the positions of dynamic obstacles.
        """
        self.obstacle_manager.update_obstacles(dt=dt)

    def process_sensor_data(self):
        """
        Read and process depth data from the vision sensor.
        Returns resized depth data and resolution.
        """
        depth_bytes, resolution = self.sim_interface.get_vision_sensor_data()
        depth_resized = self.depth_analyzer.process_depth_data(depth_bytes, resolution)
        return depth_resized, resolution

    def update_target_pose(self, uav_yaw):
        """
        Update target position and orientation based on keyboard input and UAV yaw.
        """
        _, _, current_target_yaw = self.sim_interface.sim.getObjectOrientation(
            self.sim_interface.handles['target'], -1)
        new_target_yaw = self.keyboard_controller.update_yaw(current_target_yaw)
        self.sim_interface.sim.setObjectOrientation(
            self.sim_interface.handles['target'], -1, [0, 0, new_target_yaw])

        old_pos = self.current_pos.copy()
        new_pos = self.keyboard_controller.update_position_with_yaw(old_pos, uav_yaw, self.step_size)
        self.current_pos = new_pos
        self.sim_interface.set_object_position('target', new_pos)

    def get_uav_state(self):
        """
        Retrieve current UAV position, yaw, and goal position.
        """
        uav_pos         = self.sim_interface.get_object_position('drone')
        _, _, uav_yaw   = self.sim_interface.sim.getObjectOrientation(
            self.sim_interface.handles['drone'], -1)
        goal_pos        = self.sim_interface.get_object_position('goal')
        return uav_pos, uav_yaw, goal_pos

    def detect_movement(self, uav_pos, current_uav_yaw, current_time):
        """
        Detect UAV movement and update stop state.
        Returns normalized movement vector and stop status.
        """
        uav_movement = np.zeros(3)
        has_significant_movement = False

        if self.last_uav_pos is not None and self.last_uav_yaw is not None:
            position_change = np.array(uav_pos) - np.array(self.last_uav_pos)
            axis_movements  = np.abs(position_change)
            yaw_change = abs(current_uav_yaw - self.last_uav_yaw)
            yaw_change = min(yaw_change, 2 * np.pi - yaw_change)

            has_position_movement    = any(axis_movements > self.position_threshold)
            has_yaw_movement         = yaw_change > self.yaw_threshold
            has_significant_movement = has_position_movement or has_yaw_movement

            movement_magnitude = np.linalg.norm(position_change)
            if movement_magnitude > 1e-6:
                uav_movement = position_change / movement_magnitude

            if has_significant_movement:
                self.last_movement_time = current_time
                self.consecutive_stops = 0
                if self.stop_debug:
                    print(f"[DEBUG] Movement detected: dx={axis_movements[0]:.4f}, dy={axis_movements[1]:.4f}, dz={axis_movements[2]:.4f}, dyaw={yaw_change:.4f}")
            else:
                self.consecutive_stops += 1
                if self.stop_debug:
                    print(f"[DEBUG] Stable state ({self.consecutive_stops}/{self.required_stops}): dx={axis_movements[0]:.4f}, dy={axis_movements[1]:.4f}, dz={axis_movements[2]:.4f}, dyaw={yaw_change:.4f}")
        else:
            self.last_movement_time = current_time
            self.consecutive_stops = 0

        self.last_uav_pos = uav_pos
        self.last_uav_yaw = current_uav_yaw
        return uav_movement, has_significant_movement

    def update_movement_label(self, current_time):
        """
        Update movement label based on keyboard input and stop conditions.
        """
        keys = self.keyboard_controller.keys_pressed
        is_stopped = False

        if keys.get('space', False):
            is_stopped = True
            self.consecutive_stops = self.required_stops
            if self.stop_debug:
                print("[DEBUG] Stop: Spacebar pressed")
        elif self.consecutive_stops >= self.required_stops:
            is_stopped = True
            if self.stop_debug:
                print(f"[DEBUG] Stop: Position stable for {self.consecutive_stops} frames")
        elif self.last_uav_pos is not None:
            no_movement_duration = current_time - self.last_movement_time
            if no_movement_duration > self.movement_timeout:
                is_stopped = True
                if self.stop_debug:
                    print(f"[DEBUG] Stop: No movement for {no_movement_duration:.2f} seconds")

        if is_stopped:
            self.current_label = 6
            self.current_label_str = 'stop'
        else:
            if keys['w']:
                self.current_label = 0
                self.current_label_str = 'forward'
            elif keys['s']:
                self.current_label = 1
                self.current_label_str = 'backward'
            elif keys['q']:
                self.current_label = 2
                self.current_label_str = 'up'
            elif keys['e']:
                self.current_label = 3
                self.current_label_str = 'down'
            elif keys['a']:
                self.current_label = 4
                self.current_label_str = 'left'
            elif keys['d']:
                self.current_label = 5
                self.current_label_str = 'right'
            elif keys['z']:
                self.current_label = 7
                self.current_label_str = 'rotate_left'
            elif keys['x']:
                self.current_label = 8
                self.current_label_str = 'rotate_right'

    def visualize_flight_path(self, uav_pos, goal_pos, depth_resized):
        """
        Update visualization for flight path and depth data.
        """
        if self.path_drawing is not None:
            self.sim_interface.sim.addDrawingObjectItem(self.path_drawing, uav_pos + [0, 1, 0])
        # self.visualizer.update_depth_display(depth_resized)
        # self.visualizer.update_path_display(uav_pos, goal_pos)

    def check_goal_reached(self, goal_pos):
        """
        Check if the UAV has reached the goal.
        Returns True if goal is reached, False otherwise.
        """
        current_dist = np.linalg.norm(np.array(self.current_pos[:2]) - np.array(goal_pos[:2]))
        if current_dist < 1:
            print("Goal reached!")
            return True
        return False

    def stream_frame_data(self, depth_resized, uav_pos, goal_pos, uav_movement, label_num, label_str):
        """
        Stream simulation data for the current frame.
        """
        frame_data = {
            'depth': depth_resized.copy(),
            'label': label_num,
            'label_str': label_str,
            'drone_position': uav_pos,
            'goal_position': goal_pos,
            'move_vector': uav_movement.tolist(),
        }
        self.data_streamer.stream_frame(frame_data)

    def run(self):
        """
        Run the UAV simulation loop in a modular fashion.
        """
        self.start_simulation()
        last_step_time = time.time()
        try:
            while self.sim_interface.get_simulation_time() < self.simulation_time:
                if time.time() - last_step_time < 0.2:  # Maintain 5 FPS
                    continue
                last_step_time = time.time()

                # Update simulation state
                self.update_obstacles()
                depth_resized, _            = self.process_sensor_data()
                uav_pos, uav_yaw, goal_pos  = self.get_uav_state()
                self.update_target_pose(uav_yaw)
                current_time = time.time()
                uav_movement, _ = self.detect_movement(uav_pos, uav_yaw, current_time)
                self.update_movement_label(current_time)

                # Visualize and check goal
                self.visualize_flight_path(uav_pos, goal_pos, depth_resized)
                if self.check_goal_reached(goal_pos):
                    break

                # Stream data and advance simulation
                self.stream_frame_data(
                    depth_resized, 
                    uav_pos, 
                    goal_pos, 
                    uav_movement,
                    self.current_label, 
                    self.current_label_str
                )
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