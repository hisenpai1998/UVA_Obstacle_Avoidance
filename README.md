# UAV Obstacle Avoidance (CoppeliaSim + Depth)

This project implements **UAV obstacle avoidance** using **depth images (depth maps)** in **CoppeliaSim**. The main pipeline reads depth from a vision sensor, analyzes key regions, combines it with APF-based navigation, and streams per-frame data for offline analysis/training.

The codebase is modular: CoppeliaSim (ZMQ Remote API) integration, depth processing, environment/obstacle management, keyboard control, navigation, and data streaming.

---

## 1) Requirements

### Software

- **CoppeliaSim** (with **ZMQ Remote API Server** enabled/running)
- **Python** (3.9+ recommended)

### Python dependencies

Install using `requirements.txt` (in this folder):

```bash
pip install -r requirements.txt
```

The project directly imports these third-party packages:

- `coppeliasim-zmqremoteapi-client`
- `numpy`
- `scipy` (uses `scipy.ndimage.zoom` for depth resizing)
- `matplotlib` (depth + flight path visualization)
- `pynput` (keyboard listener)

---

## 2) CoppeliaSim scene setup

1. Open the scene: `Enviroment.ttt` in CoppeliaSim.
2. Ensure the ZMQ Remote API server is running (commonly the **ZMQ remote API server** add-on).
3. By default, the code connects to `localhost:23000`.

If you need a different host/port, update where `UAVSimulator(host, port)` / `CoppeliaSimInterface(host, port)` is constructed.

### Object paths in the scene

Object paths are configured in `config.py`:

```py
OBJECT_PATHS = {
	 'target':        '/target',
	 'drone':         '/Quadcopter',
	 'goal':          '/Goal',
	 'floor':         '/Floor',
	 'vision_sensor': '/Quadcopter/visionPython'
}
```

If your scene uses different names/paths, update `OBJECT_PATHS` accordingly.

---

## 3) Run the simulation

Main entry point:

```bash
python main.py
```

High-level execution flow:

1. `UAVSimulator` connects to CoppeliaSim, enables stepping, and resolves object handles.
2. `EnvironmentGenerator.setup_environment(...)` clears the previous environment and spawns:
	- UAV + Goal (placed near opposite floor edges)
	- static trees (trunk/branches)
	- dynamic obstacles (spheroids) with randomized velocities
3. `UAVNavigator.run()` loops about every ~0.2s (~5 FPS):
	- updates obstacle states
	- reads & processes depth from the vision sensor
	- updates the target pose from keyboard input
	- streams frame data (depth + labels + poses + motion vectors)
	- advances simulation via `sim.step()`

---

## 4) Keyboard controls

While the simulation is running:

- `W` / `S`: forward / backward
- `A` / `D`: left / right (strafe)
- `Q` / `E`: up / down
- `Z` / `X`: yaw rotate left / right
- `Space`: stop
- `Esc`: stop the keyboard listener

Note: movement is computed in the drone local frame using yaw (see `keyboard_manager.py`).

---

## 5) Key configuration

All parameters are in `config.py`, including:

- `STEP_SIZE`: movement step size per update
- `SIMULATION_TIME`: simulation time limit (seconds per CoppeliaSim API)
- `DEPTH_THRESHOLD`: depth safety threshold
- `DEPTH_RESOLUTION`: resized depth resolution (currently 256×256)
- APF parameters: `ATTRACTIVE_GAIN`, `REPULSIVE_GAIN`, `REPULSIVE_RANGE`, ...
- `STREAM_FILE`: pickle output file for streaming (default `uav_stream_data.pkl`)

---

## 6) Streamed output format

Each frame is written via `pickle.dump(...)` as a dict with keys:

- `depth`: numpy array (256×256)
- `label`: numeric label (0..8)
- `label_str`: string label (`forward`, `left`, `stop`, ...)
- `drone_position`: UAV position
- `goal_position`: goal position
- `move_vector`: movement vector (from motion detection)

Default output file: `uav_stream_data.pkl` (configured by `STREAM_FILE` in `config.py`).

---

## 7) Offline playback / analysis

### Option A: Visualize frames (recommended)

```bash
python read_saved_data.py
```

This script uses a local variable `file_path` (currently set to something like `"1.pkl"`). Change it to your target stream file.

Controls in the plot window:

- `←` / `→`: previous / next frame
- `q`: quit

### Option B: Count label distribution

```bash
python check_saved_data.py
```

This script also reads `1.pkl` by default; change it similarly.

---

## 8) Troubleshooting

- **Cannot connect to CoppeliaSim**: verify CoppeliaSim is running, ZMQ Remote API server add-on is enabled, and the port matches (default `23000`).
- **Cannot resolve object handles**: scene object names/paths do not match `OBJECT_PATHS` in `config.py`.
- **Matplotlib window does not show**: ensure you are not running headless, and check matplotlib backend on Windows.
- **Pynput does not capture keys**: run in a local desktop session; some remote/virtualized setups block global keyboard hooks.

---

## License

This project is **private and proprietary**, created by **Hieu Tran Quang and Duc Huy Vu** as part of an academic course.

All rights reserved. Use, distribution, or modification without written permission is strictly prohibited. Authorized instructors/examiners may view this code for grading purposes.

See `LICENSE` for details.
