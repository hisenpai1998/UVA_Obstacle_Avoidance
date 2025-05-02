import time
import math
import numpy 				as np
import matplotlib.pyplot 	as plt
import matplotlib.patches 	as patches
import random

from scipy.ndimage 						import zoom
from pynput 							import keyboard
from coppeliasim_zmqremoteapi_client 	import RemoteAPIClient

# --- Connect to CoppeliaSim ---
client = RemoteAPIClient('localhost', 23000)
sim = client.getObject('sim')
sim.startSimulation()
print("Simulation started")

# --- Get object handles ---
target_path = "/target"
drone_path = "/Quadcopter"
goal_handle_path = "/Goal"
floor_path = "/Floor"
vision_sensor_path = "/Quadcopter/visionPython"
try:
	target_handle = sim.getObject(target_path)
	drone_handle = sim.getObject(drone_path)
	goal_handle = sim.getObject(goal_handle_path)
	floor_handle = sim.getObject(floor_path)
	vision_sensor_handle = sim.getObject(vision_sensor_path)
	sim.setExplicitHandling(vision_sensor_handle, 1)
	print(f"Handles: drone={drone_handle}, floor={floor_handle}, vision_sensor={vision_sensor_handle}")
except Exception as e:
	print(f"Error accessing objects: {e}")
	sim.stopSimulation()
	exit()

sim.setStepping(True)

# --- Initialize collision detection handle ---
collection = sim.createCollection(0)
sim.addItemToCollection(collection, sim.handle_tree, drone_handle, 0)
# Function to change color of colliding objects
def change_color(handles, color=[1, 0, 0]):
	for handle in handles:
		sim.setObjectColor(handle, 0, sim.colorcomponent_ambient_diffuse, color)

# --- Initialize dynamic obstacles 動態障礙物初始化 ---
Dynamic_handle_o = []
Dynamic_handle_b = []
obstacle_states = []
def DynamicEnv_handle():
	global Dynamic_handle_o, Dynamic_handle_b, obstacle_states
	try:
		for i in range(1, 4):
			obj_o = sim.getObject(f"/o{i}")
			obj_b = sim.getObject(f"/b{i}")
			Dynamic_handle_o.append(obj_o)
			Dynamic_handle_b.append(obj_b)
			obstacle_states.append({
				'is_moving': random.choice([True, False]),
				'next_toggle_time': time.time() + random.uniform(1, 5),
				'direction': 1 if i % 2 == 0 else -1,
				'last_offset': 0.0,
				'pause_time': 0.0
			})
			obstacle_states.append({
				'is_moving': random.choice([True, False]),
				'next_toggle_time': time.time() + random.uniform(1, 5),
				'direction': 1 if i % 2 == 0 else -1,
				'last_offset': 0.0,
				'pause_time': 0.0
			})
	except Exception as e:
		raise ValueError(f"Error accessing dynamic obstacles: {e}")
DynamicEnv_handle()
orig_positions_o = [sim.getObjectPosition(h, -1) for h in Dynamic_handle_o]
orig_positions_b = [sim.getObjectPosition(h, -1) for h in Dynamic_handle_b]
amplitude = 3
frequency = 0.5
_start_time = time.time()

# ---  Plotting setup 繪圖設定 ---
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

fig_path = plt.figure(3, figsize=(8, 6))
ax_path = fig_path.add_subplot(111)
ax_path.set_title("UAV Flight Path (APF)")
ax_path.set_xlabel("X (m)")
ax_path.set_ylabel("Y (m)")
ax_path.grid(True)
path_line, = ax_path.plot([], [], 'g-', linewidth=2, label='Flight Path')
goal_marker = ax_path.scatter([], [], c='blue', s=80, marker='*', label='Goal')
drone_marker = ax_path.scatter([], [], c='red', s=40, label='Current')
ax_path.legend()
path_x = []
path_y = []

# --- APF: Artificial Potential Field method 人工勢場法 ---
def calculate_apf_forces(current_pos, goal_pos, obstacles, d_max=0.8, K_att=0.7, K_rep=1.2):
	current_pos = np.array(current_pos)
	goal_pos = np.array(goal_pos)
	dist_to_goal = np.linalg.norm(current_pos[:2] - goal_pos[:2])
	direction_to_goal = (goal_pos[:2] - current_pos[:2]) / dist_to_goal if dist_to_goal > 1e-3 else np.zeros(2)
	f_att = K_att * np.log1p(dist_to_goal) * direction_to_goal
	f_rep = np.zeros(2)
	for obs in obstacles:
		obs_pos = np.array(obs)
		dist_to_obs = np.linalg.norm(current_pos[:2] - obs_pos[:2])
		if 1e-3 < dist_to_obs < d_max:
			direction = (current_pos[:2] - obs_pos[:2]) / dist_to_obs
			f_rep += K_rep * (1.0/dist_to_obs - 1.0/d_max) * direction
	return f_att + f_rep

# --- Obstacle avoidance using depth map 深度圖避障方向 ---
def find_safe_directions(depth_image, threshold_distance=0.6, num_sectors=16):
	height, width = depth_image.shape
	center_y, center_x = height // 2, width // 2
	sector_depths = []
	sector_angles = []
	angle_step = 2 * np.pi / num_sectors
	Y, X = np.ogrid[:height, :width]
	dist_from_center = np.sqrt((X - center_x)**2 + (Y - center_y)**2)
	max_radius = min(center_x, center_y)
	for i in range(num_sectors):
		angle = i * angle_step
		angles = np.arctan2(center_y - Y, X - center_x) % (2 * np.pi)
		sector_start = angle
		sector_end = (angle + angle_step) % (2 * np.pi)
		if sector_start < sector_end:
			in_sector = (sector_start <= angles) & (angles < sector_end)
		else:
			in_sector = (sector_start <= angles) | (angles < sector_end)
		in_sector = in_sector & (dist_from_center < max_radius)
		if np.any(in_sector):
			depths_in_sector = depth_image[in_sector]
			valid_depths = depths_in_sector[depths_in_sector > 0]
			avg_depth = np.mean(valid_depths) if len(valid_depths) > 0 else float('inf')
		else:
			avg_depth = float('inf')
		sector_depths.append(avg_depth)
		sector_angles.append(angle + angle_step/2)
	safe_sectors = [i for i, d in enumerate(sector_depths) if d > threshold_distance]
	best_sector = max(safe_sectors, key=lambda i: sector_depths[i]) if safe_sectors else np.argmax(sector_depths)
	return safe_sectors, sector_angles[best_sector], sector_depths

# --- Main loop 主循環 ---
try:
	while sim.getSimulationTime() < 300:
		t = time.time() - _start_time
		current_time = time.time()
		for idx, handle in enumerate(Dynamic_handle_o):
			state = obstacle_states[idx * 2]
			orig_pos = orig_positions_o[idx]
			if current_time >= state['next_toggle_time']:
				if state['is_moving']:
					state['last_offset'] = amplitude * math.sin(2 * math.pi * frequency * (t - state['pause_time'])) * state['direction']
					state['pause_time'] = t
				else:
					state['pause_time'] = t - state['pause_time']
				state['is_moving'] = not state['is_moving']
				state['next_toggle_time'] = current_time + random.uniform(1, 5)
			new_pos = list(orig_pos)
			offset = amplitude * math.sin(2 * math.pi * frequency * (t - state['pause_time'])) * state['direction'] if state['is_moving'] else state['last_offset']
			new_pos[0] += offset
			sim.setObjectPosition(handle, -1, new_pos)

		for idx, handle in enumerate(Dynamic_handle_b):
			state = obstacle_states[idx * 2 + 1]
			orig_pos = orig_positions_b[idx]
			if current_time >= state['next_toggle_time']:
				if state['is_moving']:
					state['last_offset'] = amplitude * math.sin(2 * math.pi * frequency * (t - state['pause_time'])) * state['direction']
					state['pause_time'] = t
				else:
					state['pause_time'] = t - state['pause_time']
				state['is_moving'] = not state['is_moving']
				state['next_toggle_time'] = current_time + random.uniform(1, 5)
			new_pos = list(orig_pos)
			offset = amplitude * math.sin(2 * math.pi * frequency * (t - state['pause_time'])) * state['direction'] if state['is_moving'] else state['last_offset']
			new_pos[2] += offset
			sim.setObjectPosition(handle, -1, new_pos)

		sim.handleVisionSensor(vision_sensor_handle)
		depth_bytes, resolution = sim.getVisionSensorDepth(vision_sensor_handle)
		resX, resY = resolution
		depth_array = np.frombuffer(depth_bytes, dtype=np.float32).reshape(resY, resX)
		depth_array = np.flipud(depth_array)
		depth_resized = zoom(depth_array, (256 / resY, 256 / resX), order=1)
		plt.figure(1)
		depth_display.set_data(depth_resized)
		depth_display.set_clim(vmin=np.min(depth_resized), vmax=np.max(depth_resized))

		safe_sectors, best_direction, sector_depths = find_safe_directions(depth_resized)

		plt.figure(2)
		if arrow:
			arrow.remove()
		for patch in sector_patches:
			patch.remove()
		sector_patches = []
		radius = 1.0
		center_x, center_y = 0, 0
		num_sectors = 16
		angle_step = 2 * np.pi / num_sectors
		for i in range(num_sectors):
			color = 'green' if i in safe_sectors else 'red'
			alpha = 0.4 if i in safe_sectors else 0.3
			wedge = patches.Wedge((center_x, center_y), radius,
								  np.degrees(i * angle_step),
								  np.degrees((i + 1) * angle_step),
								  fc=color, alpha=alpha)
			ax_obstacle.add_patch(wedge)
			sector_patches.append(wedge)
		dx = radius * 0.8 * np.cos(best_direction)
		dy = radius * 0.8 * np.sin(best_direction)
		arrow = ax_obstacle.arrow(center_x, center_y, dx, dy, head_width=0.1, head_length=0.15,
								 fc='white', ec='black', linewidth=2)
		drone_marker2 = plt.Circle((0, 0), 0.1, color='blue')
		ax_obstacle.add_patch(drone_marker2)
		sector_patches.append(drone_marker2)

		current_pos = sim.getObjectPosition(target_handle, -1)
		goal_pos = sim.getObjectPosition(goal_handle, -1)
		obstacles = [sim.getObjectPosition(h, -1) for h in Dynamic_handle_o + Dynamic_handle_b]
		min_depth = np.min(depth_resized)
		if min_depth < 0.6:
			obs_virtual = [current_pos[0] + 0.5 * np.cos(best_direction),
						   current_pos[1] + 0.5 * np.sin(best_direction),
						   current_pos[2]]
			obstacles.append(obs_virtual)
		apf_force = calculate_apf_forces(current_pos, goal_pos, obstacles)
		move_vec = np.array([apf_force[0], apf_force[1], 0])
		step_apf = 0.06
		new_target_pos = np.array(current_pos) + step_apf * move_vec
		new_target_pos[2] = max(0.2, min(2.0, new_target_pos[2]))
		sim.setObjectPosition(target_handle, -1, new_target_pos.tolist())

		path_x.append(new_target_pos[0])
		path_y.append(new_target_pos[1])
		path_line.set_data(path_x, path_y)
		drone_marker.set_offsets([[new_target_pos[0], new_target_pos[1]]])
		goal_marker.set_offsets([[goal_pos[0], goal_pos[1]]])
		ax_path.relim()
		ax_path.autoscale_view()
		plt.figure(3)
		plt.pause(0.001)

		# --- Collision detection and color change 碰撞偵測與變色 ---
		collision_status, colliding_handles = sim.checkCollision(collection, sim.handle_all)
		if collision_status:
			print(f"Collision detected: {colliding_handles}")
			change_color(colliding_handles)  # Turn colliding objects red
	
		dist_to_goal = np.linalg.norm(np.array(new_target_pos[:2]) - np.array(goal_pos[:2]))
		if dist_to_goal < 1:
			print("Goal reached!")
			break

		sim.step()

finally:
	sim.stopSimulation()
	plt.ioff()
	plt.close('all')
	print('Simulation ended.')
