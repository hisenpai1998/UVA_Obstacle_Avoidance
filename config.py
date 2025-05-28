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

"""
Essential configuration for simulation
"""	
STEP_SIZE           = 0.1           # Step size for keyboard control
NUM_SECTORS         = 16            # Number of yaw sectors for obstacle avoidance
SIMULATION_TIME     = 30000          # Simulation time in seconds
DEPTH_THRESHOLD     = 0.3           # Depth threshold for obstacle avoidance
DEPTH_RESOLUTION    = (256, 256)    # Updated to match sensor resolution

"""
Object paths matching your scene hierarchy
"""	
OBJECT_PATHS = {
    'target':           '/target',
    'drone':            '/Quadcopter',
    'goal':             '/Goal',
    'floor':            '/Floor',
    'vision_sensor':    '/Quadcopter/visionPython'
}

"""
APF parameters
"""
ATTRACTIVE_GAIN     = 1.0       # Gain for attractive force towards goal
REPULSIVE_GAIN      = 4.0       # Gain for repulsive force from obstacles
REPULSIVE_RANGE     = 2.0       # Range within which repulsive force is active
COLLISION_DISTANCE  = 0.2       # Distance threshold for collision detection

"""
Path to save the streamed data
"""
STREAM_FILE         = "uav_stream_data.pkl"
