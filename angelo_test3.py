import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pickle  # <-- ADDED: For saving data

from scipy.ndimage import zoom
from pynput import keyboard
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Create a client to connect to CoppeliaSim
client = RemoteAPIClient('localhost', 23000)
sim = client.getObject('sim')

# Start simulation
sim.startSimulation()
print("Simulation started")

# Get object handles
target_path = "/target"
drone_path = "/Quadcopter"
goal_path = "/Goal"
floor_path = "/Floor"
vision_sensor_path = "/Quadcopter/visionPython"

try:
    target_handle = sim.getObject(target_path)
    drone_handle = sim.getObject(drone_path)
    goal_handle = sim.getObject(goal_path)
    floor_handle = sim.getObject(floor_path)
    vision_sensor_handle = sim.getObject(vision_sensor_path)

    sim.setExplicitHandling(vision_sensor_handle, 1)

    print(f"Handles: drone={drone_handle}, "
          f"floor={floor_handle}, "
          f"vision_sensor={vision_sensor_handle}")

except Exception as e:
    print(f"Error accessing objects: {e}")
    sim.stopSimulation()
    exit()

# Set control variables
step_size = 0.1
current_pos = sim.getObjectPosition(target_handle, -1)
x, y, z = current_pos[0], current_pos[1], current_pos[2]
sim.setStepping(True)

# === OPEN STREAM FILE ===
stream_file = open("uav_stream_data.pkl", "ab")

# Plot setup
plt.ion()
fig_depth = plt.figure(1, figsize=(8, 6))
ax_depth = fig_depth.add_subplot(111)
depth_display = ax_depth.imshow(np.zeros((256, 256)), cmap='jet', interpolation='nearest', vmin=0, vmax=1)
cbar = fig_depth.colorbar(depth_display, ax=ax_depth)
cbar.set_label('Depth (meters)')
ax_depth.set_title("Depth Map")

fig_obstacle = plt.figure(2, figsize=(8, 6))
ax_obstacle = fig_obstacle.add_subplot(111)
ax_obstacle.set_title("Obstacle Avoidance Guidance")
ax_obstacle.set_xlim(-1.2, 1.2)
ax_obstacle.set_ylim(-1.2, 1.2)
ax_obstacle.set_aspect('equal')
ax_obstacle.grid(True)
arrow = None
sector_patches = []

keys_pressed = {'w': False, 's': False, 'a': False, 'd': False, 'q': False, 'e': False, 'space': False}

def on_press(key):
    global keys_pressed
    try:
        if key.char in keys_pressed:
            keys_pressed[key.char] = True
    except AttributeError:
        if key == keyboard.Key.space:
            keys_pressed['space'] = True
        elif key == keyboard.Key.esc:
            return False

def on_release(key):
    global keys_pressed
    try:
        if key.char in keys_pressed:
            keys_pressed[key.char] = False
    except AttributeError:
        if key == keyboard.Key.space:
            keys_pressed['space'] = False

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

def find_safe_directions(depth_image, threshold_distance=0.6, num_sectors=16):
    
    height, width = depth_image.shape
    
    # Find the center of the image
    center_y, center_x = height // 2, width // 2
    
    # Initialize lists to store sector depths and angles
    sector_depths, sector_angles = [], []

    # Calculate the angle step for each sector
    angle_step = 2 * np.pi / num_sectors

    # Create a grid of coordinates for the image
    Y, X = np.ogrid[:height, :width]
    dist_from_center = np.sqrt((X - center_x)**2 + (Y - center_y)**2)

    # Calculate the maximum radius for the sectors
    max_radius = min(center_x, center_y)

    for i in range(num_sectors):

        # Calculate the angle for the current sector
        angle = i * angle_step
        
        # Calculate the angles for all pixels in the image 
        angles = np.arctan2(center_y - Y, X - center_x) % (2 * np.pi)

        # Determine the sector boundaries
        sector_start = angle
        sector_end = (angle + angle_step) % (2 * np.pi)

        # Check if the sector is in the first or second half of the circle
        if sector_start < sector_end:
            in_sector = (sector_start <= angles) & (angles < sector_end)
        else:
            in_sector = (sector_start <= angles) | (angles < sector_end)

        # Apply the distance condition to the sector
        # Only consider pixels within the maximum radius
        in_sector = in_sector & (dist_from_center < max_radius)

        # Calculate the average depth for the sector
        # Only consider valid depth values (greater than 0)
        if np.any(in_sector):
            valid_depths = depth_image[in_sector][depth_image[in_sector] > 0]
            avg_depth = np.mean(valid_depths) if len(valid_depths) > 0 else float('inf')
        
        # If no valid depths, set to infinity
        else:
            avg_depth = float('inf')

        # Append the average depth and angle to the lists
        sector_depths.append(avg_depth)
        sector_angles.append(angle + angle_step / 2)

    # Convert to numpy arrays for easier manipulation. Goal is to find the best sector.
    #16 sectors, 0.6 threshold
    safe_sectors = [i for i, dept in enumerate(sector_depths) if dept > threshold_distance]
    if safe_sectors:
        best_sector = max(safe_sectors, key=lambda i: sector_depths[i])
    else:
        best_sector = np.argmax(sector_depths)
    return safe_sectors, sector_angles, sector_depths, best_sector, sector_angles[best_sector] # Best direction

def direction_to_label(angle, depth, safe_sectors, sector_angles, sector_depths, vertical=0, depth_threshold=0.6):
    """Convert angle to label and calculate components based on depth."""
  
    if depth < depth_threshold:
        if safe_sectors:
            # If no safe sector, find the best one
            safe_depths = [(i, sector_depths[i]) for i in safe_sectors]
            safe_depths.sort(key=lambda x: x[1], reverse=True)  # Sort by depth
            new_angle = sector_angles[safe_depths[0][0]] # Best safe sector angle
            new_depth = sector_depths[safe_depths[0][0]] # Best safe sector depth

            # Calculate components based on new angle and depth
            x = new_depth * np.cos(new_angle)
            y = new_depth * np.sin(new_angle)
            z = vertical * new_depth
            components = {
                'right': x,
                'left': -x,
                'forward': y,
                'backward': -y,
                'up': z,
                'down': -z
            }
            label = max(components, key=lambda k: abs(components[k]))
            return label, new_angle
        else:
            # if no safe sectors, return a default label
            if vertical == 0:
                return "up", angle  # Default to "up" if no safe sectors and vertical is 0
            return "stop", angle
    
    # If find safe sectors so used the best sector angle and depth
    x = depth * np.cos(angle)
    y = depth * np.sin(angle)
    z = vertical * depth
    components = {
        'right': x,
        'left': -x,
        'forward': y,
        'backward': -y,
        'up': z,
        'down': -z
    }
    label = max(components, key=lambda k: abs(components[k]))
    return label, angle

try:
    while sim.getSimulationTime() < 300:
        # Handle vision sensor (depth viewport data)
        sim.handleVisionSensor(vision_sensor_handle)
        depth_bytes, resolution = sim.getVisionSensorDepth(vision_sensor_handle)
        resX, resY = resolution

        # Process depth data
        depth_array = np.frombuffer(depth_bytes, dtype=np.float32).reshape(resY, resX)
        depth_array = np.flipud(depth_array)

        # Dynamically scale to avoid hardcoded resolution
        depth_resized = zoom(depth_array, (256 / resY, 256 / resX), order=1)

        # Update depth viewport display
        plt.figure(1)
        depth_display.set_data(depth_resized)
        depth_display.set_clim(vmin=np.min(depth_resized), vmax=np.max(depth_resized))

        # Analyze a depth map and find safe path
        safe_sectors, sector_angles, sector_depths, best_sector, best_direction = find_safe_directions(depth_resized, threshold_distance=0.4)

        # Define label and positions
        label = direction_to_label(best_direction, sector_depths[best_sector], safe_sectors, sector_angles, sector_depths, vertical=0, depth_threshold=0.4)  # Use sector_depths[best_sector]
        #label = direction_to_label(best_direction)
        uav_pos = sim.getObjectPosition(drone_handle, -1)
        goal_pos = sim.getObjectPosition(goal_handle, -1)

        # === STREAM EACH FRAME TO PKL ===
        frame_data = {
            'depth': depth_resized.copy(),
            'label': label,
            'uav_pos': uav_pos,
            'goal_pos': goal_pos,
            'best_direction': best_direction
        }
        pickle.dump(frame_data, stream_file)  # Stream frame to file

        # Update obstacle avoidance window
        plt.figure(2)  # Switch to obstacle guidance window

        # Remove old visual elements
        if arrow is not None:
            arrow.remove()
        for patch in sector_patches:
            patch.remove()
        sector_patches = []

        # Visualize safe paths and the best direction
        radius = 1.0
        center_x, center_y = 0, 0

        # Draw sectors
        num_sectors = 16
        angle_step = 2 * np.pi / num_sectors

        for i in range(num_sectors):
            # Color based on depth
            if i in safe_sectors:
                color = 'green'  # Green for safe
                alpha = 0.4
            else:
                color = 'red'  # Red for unsafe
                alpha = 0.3

            # Create sector
            wedge = patches.Wedge(
                (center_x, center_y),
                radius,
                np.degrees(i * angle_step),
                np.degrees((i + 1) * angle_step),
                fc=color,
                alpha=alpha
            )
            ax_obstacle.add_patch(wedge)
            sector_patches.append(wedge)

        # Draw the best direction arrow
        dx = radius * 0.8 * np.cos(best_direction)
        dy = radius * 0.8 * np.sin(best_direction)
        arrow = ax_obstacle.arrow(center_x, center_y, dx, dy, head_width=0.1, head_length=0.15,
                                  fc='white', ec='black', linewidth=2)

        # Add drone position marker
        drone_marker = plt.Circle((0, 0), 0.1, color='blue')
        ax_obstacle.add_patch(drone_marker)
        sector_patches.append(drone_marker)

        # Update figures
        plt.pause(0.001)

        # Get current target position
        current_pos = sim.getObjectPosition(target_handle, -1)
        x, y, z = current_pos[0], current_pos[1], current_pos[2]

        # Update position based on keyboard input
        if keys_pressed['w']:  # Forward (+y)
            y += step_size
        if keys_pressed['s']:  # Backward (-y)
            y -= step_size
        if keys_pressed['a']:  # Left (-x)
            x -= step_size
        if keys_pressed['d']:  # Right (+x)
            x += step_size
        if keys_pressed['q']:  # Up (+z)
            z += step_size
        if keys_pressed['e']:  # Down (-z)
            z -= step_size
        if keys_pressed['space']:  # Reset to the initial position
            x, y, z = 0, 0, 0.5  # Example reset position

        # Set a new target position
        sim.setObjectPosition(target_handle, -1, [x, y, z])

        # Check if the goal is reached
        target_pos = sim.getObjectPosition(target_handle, -1)
        goal_pos = sim.getObjectPosition(goal_handle, -1)
        current_dist = np.linalg.norm(np.array(target_pos[:2]) - np.array(goal_pos[:2]))
        if current_dist < 1:
            print("Goal reached!")
            break

        sim.step()

finally:
    listener.stop()
    sim.stopSimulation()
    stream_file.close()  # === CLOSE STREAM FILE ===
    plt.ioff()
    plt.close('all')

    print('Simulation ended and data saved.')

def read_saved_data(file_path="uav_stream_data.pkl"):
    """Reads and prints data from the saved UAV stream file."""
    with open(file_path, "rb") as f:
        print("Reading saved UAV data...")
        try:
            while True:
                # Load each frame from the file
                frame_data = pickle.load(f)
                print("Label:", frame_data["label"])
                print("UAV Position:", frame_data["uav_pos"])
                print("Goal Position:", frame_data["goal_pos"])
                print("Best Direction (rad):", frame_data["best_direction"])
                print("Depth Sample (top-left 2x2):")
                print(frame_data["depth"][:2, :2])
                print("---")
        except EOFError:
            # End of file reached
            print("Finished reading all frames.")

# Call the function to test it
if __name__ == "__main__":
    read_saved_data()