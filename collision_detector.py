import numpy    as np
from config     import COLLISION_DISTANCE

class CollisionDetector:
    def __init__(self):
        """Initialize the collision detector."""
        pass

    def check_collision(self, uav_pos, obstacle_positions):
        """Check if the UAV collides with any obstacle."""
        uav_pos = np.array(uav_pos[:2])
        for obs_pos in obstacle_positions.values():
            obs_pos     = np.array(obs_pos[:2])
            distance    = np.linalg.norm(uav_pos - obs_pos)
            if distance < COLLISION_DISTANCE:
                return True
        return False