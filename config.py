# Essential configuration for simulation
STEP_SIZE           = 0.1           # Step size for keyboard control
NUM_SECTORS         = 16            # Number of yaw sectors for obstacle avoidance
SIMULATION_TIME     = 3000         # Simulation time in seconds
DEPTH_THRESHOLD     = 0.3           # Depth threshold for obstacle avoidance
DEPTH_RESOLUTION    = (256, 256)    # Updated to match sensor resolution

# Object paths matching your scene hierarchy
OBJECT_PATHS = {
    'target':           '/target',
    'drone':            '/Quadcopter',
    'goal':             '/Goal',
    'floor':            '/Floor',
    'vision_sensor':    '/Quadcopter/visionPython',

    'obstacles': ['/Cuboid[0]', '/Cuboid[1]', '/Cuboid[2]', 
                 '/Cuboid[3]', '/Cuboid[4]', 
                 '/Cuboid[5]', '/Cuboid[6]'],
                 
    'walls': ['/w1', '/w2', '/w3', '/w4', '/w5']  # Adding wall objects
}

# APF parameters
ATTRACTIVE_GAIN     = 1.0       # Gain for attractive force towards goal
REPULSIVE_GAIN      = 4.0       # Gain for repulsive force from obstacles
REPULSIVE_RANGE     = 2.0       # Range within which repulsive force is active
COLLISION_DISTANCE  = 0.2       # Distance threshold for collision detection

# Path to save the streamed data
STREAM_FILE         = "uav_stream_data.pkl"
