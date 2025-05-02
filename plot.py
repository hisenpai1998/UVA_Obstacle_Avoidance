import matplotlib.pyplot    as plt
import matplotlib.patches   as patches
import numpy                as np

class Visualizer:
    def __init__(self):
        """Initialize visualization windows."""
        # Depth Map (Figure 1)
        self.fig_depth      = None
        self.ax_depth       = None
        self.depth_display  = None
        self.cbar           = None

        # Flight Path (Figure 2)
        self.fig_path       = None
        self.ax_path        = None
        self.path_line      = None
        self.goal_marker    = None
        self.drone_marker   = None
        self.path_x        = []
        self.path_y        = []

        # Obstacle Avoidance (Figure 3)
        self.fig_obstacle   = None
        self.ax_obstacle    = None
        self.arrow          = None
        self.sector_patches  = []

        # Set up the visualization windows
        self._initialize()

    def _initialize(self):
        """Set up the visualization windows."""
        plt.ion()

        #------------------------------------------------------------------
        # Depth Map (Figure 1)
        self.fig_depth      = plt.figure(1, figsize=(8, 6))
        self.ax_depth       = self.fig_depth.add_subplot(111)
        self.depth_display  = self.ax_depth.imshow(
            np.zeros((256, 256)), 
            cmap='jet', 
            interpolation='nearest',    
            vmin=0, 
            vmax=1
        )
        self.cbar = self.fig_depth.colorbar(self.depth_display, ax=self.ax_depth)
        self.cbar.set_label('Depth (meters)')
        self.ax_depth.set_title("Depth Map")
        self.ax_depth.set_aspect('equal')

        # Draw grid lines on the depth map
        h, w = 256, 256  # Assuming the depth map is 256x256 pixels
        self.grid_lines = []

        # 2 line cutting the image into 3 rows
        for i in [1, 2]:
            y    = i * h / 3
            line = self.ax_depth.axhline(y=y, color='white', linestyle='--', linewidth=1)
            self.grid_lines.append(line)

        # 2 line cutting the image into 3 columns
        for i in [1, 2]:
            x    = i * w / 3
            line = self.ax_depth.axvline(x=x, color='white', linestyle='--', linewidth=1)
            self.grid_lines.append(line)

        #------------------------------------------------------------------
        # Flight Path (Figure 2)
        self.fig_path = plt.figure(2, figsize=(8, 6))
        self.ax_path = self.fig_path.add_subplot(111)
        self.ax_path.set_title("UAV Flight Path (APF)")
        self.ax_path.set_xlabel("X (m)")
        self.ax_path.set_ylabel("Y (m)")
        self.ax_path.grid(True)
        self.path_line, = self.ax_path.plot([], [], 'g-', linewidth=2, label='Flight Path')
        self.goal_marker = self.ax_path.scatter([], [], c='blue', s=80, marker='*', label='Goal')
        self.drone_marker = self.ax_path.scatter([], [], c='red', s=40, label='Current')
        self.ax_path.legend()

        # Obstacle Avoidance (Figure 3)
        # self.fig_obstacle = plt.figure(3, figsize=(8, 6))
        # self.ax_obstacle = self.fig_obstacle.add_subplot(111)
        # self.ax_obstacle.set_title("Obstacle Avoidance Guidance")
        # self.ax_obstacle.set_xlim(-1.2, 1.2)
        # self.ax_obstacle.set_ylim(-1.2, 1.2)
        # self.ax_obstacle.set_aspect('equal')
        # self.ax_obstacle.grid(True)
        # self.arrow = None
        # self.sector_patches = []


    def update_depth_display(self, depth_data):
        """Update the depth map visualization."""
        plt.figure(1)
        self.depth_display.set_data(depth_data)
        self.depth_display.set_clim(vmin=np.min(depth_data), vmax=np.max(depth_data))
        plt.draw()
        plt.pause(0.001)

    def update_path_display(self, uav_pos, goal_pos):
        """Update the flight path visualization."""
        plt.figure(2)
        
        # Append the current UAV position to the path
        self.path_x.append(uav_pos[0])
        self.path_y.append(uav_pos[1])

        # Update the flight path line
        self.path_line.set_data(self.path_x, self.path_y)

        # Update the UAV's current position marker
        self.drone_marker.set_offsets([uav_pos[0], uav_pos[1]])

        # Update the goal position marker
        self.goal_marker.set_offsets([goal_pos[0], goal_pos[1]])

        # Adjust plot limits dynamically
        all_x = self.path_x + [goal_pos[0]]
        all_y = self.path_y + [goal_pos[1]]
        if all_x and all_y:
            self.ax_path.set_xlim(min(all_x) - 1, max(all_x) + 1)
            self.ax_path.set_ylim(min(all_y) - 1, max(all_y) + 1)

        plt.pause(0.001)

    def update_obstacle_display(self, safe_sectors, best_sector, best_angle):
        """Update the obstacle avoidance visualization."""
        plt.figure(3)
        if self.arrow is not None:
            self.arrow.remove()
        for patch in self.sector_patches:
            patch.remove()
        self.sector_patches = []

        radius              = 1.0
        center_x, center_y  = 0, 0
        num_sectors         = 16
        angle_step          = 2 * np.pi / num_sectors   

        for i in range(num_sectors):
            color = 'green' if i in safe_sectors else 'red'
            alpha = 0.4 if i in safe_sectors else 0.3
            wedge = patches.Wedge(
                (center_x, center_y), radius,
                np.degrees(i * angle_step), np.degrees((i + 1) * angle_step),
                fc=color, alpha=alpha
            )
            self.ax_obstacle.add_patch(wedge)
            self.sector_patches.append(wedge)

        dx = radius * 0.8 * np.cos(best_angle)
        dy = radius * 0.8 * np.sin(best_angle)
        self.arrow = self.ax_obstacle.arrow(center_x, center_y, dx, dy, 
                                            head_width=0.1, head_length=0.15,
                                            fc='white', ec='black', 
                                            linewidth=2)

        drone_marker = plt.Circle((0, 0), 0.1, color='blue')
        self.ax_obstacle.add_patch(drone_marker)
        self.sector_patches.append(drone_marker)
        plt.pause(0.001)

    def close(self):
        """Close all visualization windows."""
        plt.ioff()
        plt.close('all')