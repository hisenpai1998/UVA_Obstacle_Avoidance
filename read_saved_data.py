import pickle
import matplotlib.pyplot    as plt
import numpy                as np

# Path to the saved file
file_path = "uav_stream_data.pkl"

# Load all frames into memory
frames = []
with open(file_path, "rb") as f:
    print("Reading saved UAV data...")
    try:
        while True:
            frame_data = pickle.load(f)
            frames.append(frame_data)
    except EOFError:
        print("Finished reading all frames.")

# Initialize frame index
current_frame = 0

# Function to update the plot
def update_plot():
    global current_frame
    frame_data = frames[current_frame]

    # Plot depth map
    plt.figure(1)
    plt.clf()
    plt.imshow(frame_data["depth"], cmap="jet", interpolation="nearest")
    plt.colorbar(label="Depth (meters)")
    plt.title(f"Depth Map - Frame {current_frame + 1}/{len(frames)} - Label: {frame_data['label']}")
    plt.draw()

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

# Plot the first frame
update_plot()

# Connect the key press event to the figure
plt.gcf().canvas.mpl_connect("key_press_event", on_key)

# Show the plot
plt.show()
