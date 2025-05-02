import numpy as np
from config import ATTRACTIVE_GAIN, REPULSIVE_GAIN, REPULSIVE_RANGE

class APFNavigator:
    def __init__(self):
        """Initialize the APF navigator."""
        pass

    def calculate_force(self, uav_pos, goal_pos, obstacle_positions):
        """Calculate the total force using APF."""
        uav_pos     = np.array(uav_pos[:2])  # Use only x, y for 2D navigation
        goal_pos    = np.array(goal_pos[:2])

        # Attractive force towards the goal
        attractive_force = ATTRACTIVE_GAIN * (goal_pos - uav_pos)

        # Repulsive forces from obstacles
        repulsive_force = np.zeros(2)
        for obs_pos in obstacle_positions.values():
            obs_pos     = np.array(obs_pos[:2])
            distance    = np.linalg.norm(uav_pos - obs_pos)
            if distance < REPULSIVE_RANGE and distance > 0:
                direction   = (uav_pos - obs_pos) / distance
                repulsive_force += REPULSIVE_GAIN * (1/distance - 1/REPULSIVE_RANGE) * (1/(distance**2)) * direction

        # Total force
        total_force = attractive_force + repulsive_force
        return total_force / np.linalg.norm(total_force) if np.linalg.norm(total_force) > 0 else np.zeros(2)