import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import zoom
from pynput import keyboard
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import matplotlib.patches as patches
import pickle  # <-- ADDED: For saving data

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

def find_safe_directions(depth_image, threshold_distance, num_yaw=16, num_pitch=8):
    """Analyze depth image to find safe directions in 3D (yaw and pitch)."""
    height, width = depth_image.shape
    center_y, center_x = height // 2, width // 2
    sector_depths = np.zeros((num_pitch, num_yaw))  # 2D array for pitch × yaw
    sector_yaw_angles = []
    sector_pitch_angles = []
    safe_sectors = []  # List of (pitch_idx, yaw_idx) for safe sectors

    # Yaw (horizontal) and pitch (vertical) angle steps
    yaw_step = 2 * np.pi / num_yaw  # 360° / num_yaw
    pitch_step = np.pi / num_pitch  # 180° / num_pitch (from -90° to +90°)
    Y, X = np.ogrid[:height, :width]
    dist_from_center = np.sqrt((X - center_x)**2 + (Y - center_y)**2)
    max_radius = min(center_x, center_y)

    # Calculate angles for each pixel
    yaw_angles = np.arctan2(center_y - Y, X - center_x) % (2 * np.pi)
    pitch_angles = np.arctan2(center_y - Y, dist_from_center)  # From -π/2 to π/2

    for pitch_idx in range(num_pitch):
        pitch_angle = -np.pi / 2 + pitch_idx * pitch_step + pitch_step / 2
        sector_pitch_angles.append(pitch_angle)
        yaw_row = []
        for yaw_idx in range(num_yaw):
            yaw_angle = yaw_idx * yaw_step + yaw_step / 2
            if pitch_idx == 0:  # Only append yaw angles once
                sector_yaw_angles.append(yaw_angle)

            # Define sector boundaries
            yaw_start = yaw_idx * yaw_step
            yaw_end = (yaw_idx + 1) * yaw_step % (2 * np.pi)
            pitch_start = -np.pi / 2 + pitch_idx * pitch_step
            pitch_end = -np.pi / 2 + (pitch_idx + 1) * pitch_step

            # Select pixels in the sector
            in_sector = (
                ((yaw_start <= yaw_angles) & (yaw_angles < yaw_end)) &
                ((pitch_start <= pitch_angles) & (pitch_angles < pitch_end)) &
                (dist_from_center < max_radius)
            )

            # Calculate average depth
            valid_depths = depth_image[in_sector][depth_image[in_sector] > 0]
            avg_depth = np.mean(valid_depths) if len(valid_depths) > 0 else float('inf')
            sector_depths[pitch_idx, yaw_idx] = avg_depth

            # Mark safe sectors
            if avg_depth > threshold_distance:
                safe_sectors.append((pitch_idx, yaw_idx))

        # Find best sector (deepest depth)
        best_sector = np.unravel_index(np.argmin(sector_depths, axis=None), sector_depths.shape)
        if not safe_sectors:
            safe_sectors.append(best_sector)  # Use deepest sector if none are safe

    # Compute best direction (yaw angle of the best sector)
    best_yaw_idx = best_sector[1]  # Yaw index of the best sector
    best_direction = sector_yaw_angles[best_yaw_idx]

    return safe_sectors, sector_yaw_angles, sector_pitch_angles, sector_depths, best_sector, best_direction

def compute_goal_direction(uav_pos, goal_pos):
    """Calculate yaw and pitch angles from UAV to goal."""
    dx = goal_pos[0] - uav_pos[0]
    dy = goal_pos[1] - uav_pos[1]
    dz = goal_pos[2] - uav_pos[2]
    horizontal_dist = np.sqrt(dx**2 + dy**2)
    yaw = np.arctan2(dy, dx) % (2 * np.pi)  # Horizontal angle
    pitch = np.arctan2(dz, horizontal_dist)  # Vertical angle
    return yaw, pitch

def direction_to_label(safe_sectors, sector_yaw_angles, sector_pitch_angles, sector_depths, goal_yaw, goal_pitch, min_depth=0.1):
    """Select the safest sector closest to the goal direction and return label and angle."""
    # Validate inputs
    if sector_depths.size == 0:
        return "stop", goal_yaw

    # Check for invalid depths (NaN or negative, excluding inf)
    finite_depths = sector_depths[np.isfinite(sector_depths)]
    if (finite_depths < 0).any():
        return "stop", goal_yaw

    # Initialize best sector and score
    best_score = float('inf')
    best_label = "stop"
    best_sector = None
    best_angle = goal_yaw

    # Movement priorities (higher value = higher priority)
    priorities = {
        'forward': 3,
        'up': 2,
        'right': 1,
        'left': 1,
        'backward': 0,
        'down': 0,
        'stop': -1
    }

    for pitch_idx, yaw_idx in safe_sectors:
        depth = sector_depths[pitch_idx, yaw_idx]
        if depth < min_depth or depth == float('inf'):
            continue

        yaw = sector_yaw_angles[yaw_idx]
        pitch = sector_pitch_angles[pitch_idx]

        # Calculate angular difference from goal
        yaw_diff = min((yaw - goal_yaw) % (2 * np.pi), (goal_yaw - yaw) % (2 * np.pi))
        pitch_diff = abs(pitch - goal_pitch)
        angular_diff = yaw_diff + pitch_diff  # Simple combined metric

        # Determine movement label
        x = depth * np.cos(yaw) * np.cos(pitch)
        y = depth * np.sin(yaw) * np.cos(pitch)
        z = depth * np.sin(pitch)
        components = {
            'right': x,
            'left': -x,
            'forward': y,
            'backward': -y,
            'up': z,
            'down': -z
        }
        label = max(components, key=lambda k: abs(components[k]))

        # Calculate score: lower angular difference and higher priority = better
        score = angular_diff - priorities[label] * 0.1  # Small weight for priority

        if score < best_score:
            best_score = score
            best_label = label
            best_sector = (pitch_idx, yaw_idx)
            best_angle = yaw

    return best_label, best_angle

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
        safe_sectors, sector_yaw_angles, sector_pitch_angles, sector_depths, best_sector, best_direction = find_safe_directions(depth_resized, threshold_distance=0.4)
        
        uav_pos = sim.getObjectPosition(drone_handle, -1)
        goal_pos = sim.getObjectPosition(goal_handle, -1)
        
        goal_yaw, goal_pitch = compute_goal_direction(uav_pos, goal_pos)

        # Define label and positions
        label = direction_to_label(safe_sectors, sector_yaw_angles, sector_pitch_angles, sector_depths, goal_yaw, goal_pitch, min_depth=0.1)  # Use sector_depths[best_sector]

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