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


import pickle
import matplotlib.pyplot as plt
import numpy as np

# Path to the saved file
# file_path = "uav_stream_data.pkl"
file_path = "1.pkl"
# Load all frames into memory
frames = []
try:
    with open(file_path, "rb") as f:
        print("Reading saved UAV data...")
        while True:
            try:
                frame_data = pickle.load(f)
                frames.append(frame_data)
            except EOFError:
                print(f"Finished reading {len(frames)} frames.")
                break
except FileNotFoundError:
    print(f"Error: File {file_path} not found.")
    exit(1)
except Exception as e:
    print(f"Error reading file: {e}")
    exit(1)

if not frames:
    print("Error: No frames loaded from file.")
    exit(1)

# Initialize frame index
current_frame = 0

# Initialize figure for depth map and flight path
plt.ion()  # Enable interactive mode
fig = plt.figure(figsize=(12, 5))

# Depth map (left subplot)
ax_depth = fig.add_subplot(121)
depth_display = ax_depth.imshow(
    frames[0]["depth"], cmap="gray", interpolation="nearest", vmin=0, vmax=1
)
cbar = fig.colorbar(depth_display, ax=ax_depth, label="Depth (meters)")
ax_depth.set_title(f"Depth Map - Frame 1/{len(frames)}")
ax_depth.set_aspect("equal")

# Flight path (right subplot)
ax_path = fig.add_subplot(122)
ax_path.set_title("UAV Flight Path")
ax_path.set_xlabel("X (m)")
ax_path.set_ylabel("Y (m)")
ax_path.grid(True)
path_dots = ax_path.scatter([], [], c="green", s=10, label="Flight Path")
drone_marker = ax_path.scatter([], [], c="red", s=40, label="UAV")
goal_marker = ax_path.scatter([], [], c="blue", s=80, marker="*", label="Goal")
ax_path.legend()
path_x, path_y = [], []

# Function to update the plot
def update_plot():
    global current_frame, path_x, path_y
    frame_data = frames[current_frame]


    # Update depth map
    depth_display.set_data(frame_data["depth"])
    depth_display.set_clim(vmin=np.min(frame_data["depth"]), vmax=np.max(frame_data["depth"]))
    ax_depth.set_title(
        f"Depth Map - Frame {current_frame + 1}/{len(frames)} - Label: {frame_data['label_str']} ({frame_data['label']})"
    )

    # Update flight path
    uav_pos = frame_data["drone_position"]
    goal_pos = frame_data["goal_position"]
    path_x.append(uav_pos[0])
    path_y.append(uav_pos[1])
    path_dots.set_offsets(np.c_[path_x, path_y])
    drone_marker.set_offsets([uav_pos[0], uav_pos[1]])
    goal_marker.set_offsets([goal_pos[0], goal_pos[1]])

    # Update path axes limits
    all_x = path_x + [goal_pos[0]]
    all_y = path_y + [goal_pos[1]]
    if all_x and all_y:
        ax_path.set_xlim(min(all_x) - 1, max(all_x) + 1)
        ax_path.set_ylim(min(all_y) - 1, max(all_y) + 1)

    # Print additional info to console
    print(f"\nFrame {current_frame + 1}/{len(frames)}")
    print(f"Label: {frame_data['label_str']} ({frame_data['label']})")
    print(f"UAV Position: {frame_data['drone_position']}")
    print(f"Goal Position: {frame_data['goal_position']}")
    print(f"Move Vector: {frame_data['move_vector']}")

    fig.canvas.draw_idle()
    fig.canvas.flush_events()

# Function to handle key press events
def on_key(event):
    global current_frame
    if event.key == "right":  # Next frame
        if current_frame < len(frames) - 1:
            current_frame += 1
            update_plot()
    elif event.key == "left":  # Previous frame
        if current_frame > 0:
            current_frame -= 1
            update_plot()
    elif event.key == "q":  # Quit
        plt.close()
        exit(0)

# Plot the first frame
try:
    update_plot()
except KeyError as e:
    print(f"Error: Missing key {e} in frame data. Check data structure.")
    exit(1)

# Connect the key press event to the figure
fig.canvas.mpl_connect("key_press_event", on_key)

# Keep the plot open
try:
    plt.show(block=True)
except Exception as e:
    print(f"Error displaying plot: {e}")