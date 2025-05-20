from config     import ATTRACTIVE_GAIN, REPULSIVE_GAIN, REPULSIVE_RANGE
import numpy    as np


class APFNavigator:
    def __init__(self, floor_repulsion=True, floor_k=1.0, floor_safe_height=0.5, floor_z=0.0):

        """ 
        Initialize the APF navigator with parameters for floor repulsion.
        """

        self.floor_repulsion    = floor_repulsion
        self.floor_k            = floor_k
        self.floor_safe_height  = floor_safe_height
        self.floor_z            = floor_z

    def calculate_force(self, uav_pos, goal_pos, obstacle_positions, wall_positions=None):

        """
        Calculate the total force using APF in 3D.
        """

        uav_pos     = np.array(uav_pos)
        goal_pos    = np.array(goal_pos)

        # Attractive force towards the goal
        distance_to_goal = np.linalg.norm(goal_pos - uav_pos)
        attractive_force = ATTRACTIVE_GAIN * (goal_pos - uav_pos) / distance_to_goal if distance_to_goal > 0 else np.zeros(3)

        # Repulsive force from obstacles
        repulsive_force = np.zeros(3)
        for obs_pos in obstacle_positions:
            obs_pos     = np.array(obs_pos)
            distance    = np.linalg.norm(uav_pos - obs_pos)
            if 0 < distance < REPULSIVE_RANGE:
                direction = (uav_pos - obs_pos) / distance
                repulsive_force += REPULSIVE_GAIN * (1 / distance - 1 / REPULSIVE_RANGE) * (1 / distance**2) * direction

        # Repulsive force from walls (same as obstacle)
        if wall_positions:
            for wall_pos in wall_positions:
                wall_pos     = np.array(wall_pos)
                distance     = np.linalg.norm(uav_pos - wall_pos)
                if 0 < distance < REPULSIVE_RANGE:
                    direction = (uav_pos - wall_pos) / distance
                    repulsive_force += REPULSIVE_GAIN * (1 / distance - 1 / REPULSIVE_RANGE) * (1 / distance**2) * direction

        # Floor repulsion
        floor_force = np.zeros(3)
        if self.floor_repulsion:
            z_diff = uav_pos[2] - self.floor_z
            if z_diff < self.floor_safe_height:
                floor_force[2] = self.floor_k / z_diff if z_diff > 0 else float('inf')

        # Total force
        total_force = attractive_force + repulsive_force + floor_force
        norm        = np.linalg.norm(total_force)
        return total_force / norm if norm > 0 else np.zeros(3)