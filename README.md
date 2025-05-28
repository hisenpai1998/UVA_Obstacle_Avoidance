# UAV Obstacle Avoidance

This project implements an **intelligent obstacle avoidance system** for Unmanned Aerial Vehicles (UAVs) using **depth images** and simulation in **CoppeliaSim**.

> Fully written in Python  
> Real-time perception using RGB-D  
> Modular architecture for simulation, depth processing, collision detection, and navigation

---

## Project Goal

To enable a UAV to **autonomously avoid static and dynamic obstacles** using depth images and AI-based decision making.  
The system analyzes depth data from simulation, detects obstacles in real-time, and decides the best direction to avoid collisions.

---

## Core Features

- **Depth Image Processing** from RGB-D simulation
- **Obstacle Detection** with 9-region segmentation
- **Navigation Decision** (go left, right, or straight)
- **Simulation Integration** using CoppeliaSim API (ZMQ)
- **Modular Codebase** for extensibility and testing

---

## Code Structure

```bash
UVA_Obstacle_Avoidance/
│
├── main.py                  # Main entry point to run the system
├── config.py                # Configurations and thresholds
├── depth_processor.py       # Resize + normalize depth data
├── collision_detector.py    # Obstacle detection using depth threshold
├── UAV_navigator.py         # Decision logic (avoidance directions)
├── coppeliasim_interface.py # Interaction with CoppeliaSim (ZMQ API)
├── stream_manager.py        # Depth data stream management
├── obstacle_manager.py      # Manage obstacle history
├── UAV_simulator.py         # UAV movement and update
├── keyboard_manager.py      # Manual key control (optional)
├── plot.py                  # For visualization
├── check_saved_data.py      # Check datasets information
├── read_saved_data.py       # To load recorded data for offline analysis
├── APF_navigator.py         # Experimental: Artificial Potential Field method
└── README.md                # You're reading this file


## License

This project is **private and proprietary**, created by **Hieu Tran Quang and Huy Duc Vu** as part of an academic course.

🔒 **All rights reserved.**  
Use, distribution, or modification without written permission is strictly prohibited.

Authorized instructors or examiners may view this code for grading purposes.

See the [LICENSE](./LICENSE) file for details.
