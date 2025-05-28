# -------------------------------------------------------------------------
# Copyright (c) 2025 Hieu Tran Quang and Duc Huy Vu
# All rights reserved.
#
# This source code is part of a private academic project submitted for
# educational purposes only. It may be viewed and assessed by authorized
# instructors and examiners as part of academic evaluation.
#
# Unauthorized use, reproduction, distribution, or modification of this
# code, in whole or in part, is strictly prohibited without the prior
# written consent of the authors.
#
# This file and project are NOT open source.
# -------------------------------------------------------------------------

import numpy    as np
from config     import COLLISION_DISTANCE

class CollisionDetector:
    def __init__(self):

        """
        Initialize the collision detector.
        """

        pass

    def check_collision(self, uav_pos, obstacle_positions):

        """
        Check if the UAV collides with any obstacle.
        """
        
        uav_pos = np.array(uav_pos[:2])
        for obs_pos in obstacle_positions:
            obs_pos     = np.array(obs_pos[:2])
            distance    = np.linalg.norm(uav_pos - obs_pos)
            if distance < COLLISION_DISTANCE:
                return True
        return False