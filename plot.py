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

import matplotlib.pyplot    as plt
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
        self.path_x         = []
        self.path_y         = []

        # Set up the visualization windows
        self._initialize()

    def _initialize(self):
        """Set up the visualization windows."""

        plt.ion()

        # Depth Map (Figure 1)
        self.fig_depth      = plt.figure(1, figsize=(8, 6))
        self.ax_depth       = self.fig_depth.add_subplot(111)
        self.depth_display  = self.ax_depth.imshow(
            np.zeros((256, 256)), 
            cmap='jet', 
            interpolation='nearest',    
            vmin=0, 
            vmax=1,
            animated=True
        )
        self.cbar = self.fig_depth.colorbar(self.depth_display, ax=self.ax_depth)
        self.cbar.set_label('Depth (meters)')
        self.ax_depth.set_title("Depth Map")
        self.ax_depth.set_aspect('equal')

        h, w = 256, 256
        self.grid_lines = []
        for i in [1, 2]:
            y       = i * h / 3
            line    = self.ax_depth.axhline(y=y, color='white', linestyle='--', linewidth=1, animated=True)
            self.grid_lines.append(line)
        for i in [1, 2]:
            x       = i * w / 3
            line    = self.ax_depth.axvline(x=x, color='white', linestyle='--', linewidth=1, animated=True)
            self.grid_lines.append(line)

        # Flight Path (Figure 2)
        self.fig_path   = plt.figure(2, figsize=(8, 6))
        self.ax_path    = self.fig_path.add_subplot(111)
        self.ax_path.set_title("UAV Flight Path (APF)")
        self.ax_path.set_xlabel("X (m)")
        self.ax_path.set_ylabel("Y (m)")
        self.ax_path.grid(True)
        self.path_dots      = self.ax_path.scatter([], [], c='green', s=10, label='Flight Path')
        self.goal_marker    = self.ax_path.scatter([], [], c='blue', s=80, marker='*', label='Goal')
        self.drone_marker   = self.ax_path.scatter([], [], c='red', s=40, label='Current')
        self.ax_path.legend()

    def update_depth_display(self, depth_data):
        if not hasattr(self, '_frame_count_depth'):
            self._frame_count_depth = 0
        self._frame_count_depth += 1
        if self._frame_count_depth % 1 != 0:
            return
        self.depth_display.set_data(depth_data)
        self.depth_display.set_clim(vmin=np.min(depth_data), vmax=np.max(depth_data))
        self.fig_depth.canvas.draw_idle()
        self.fig_depth.canvas.flush_events()

    def update_path_display(self, uav_pos, goal_pos, label_str='stop'):
        self.path_x.append(uav_pos[0])
        self.path_y.append(uav_pos[1])
        
        # Draw flight path
        self.path_dots.set_offsets(np.c_[self.path_x, self.path_y])
        
        # Update drone and goal marker
        self.drone_marker.set_offsets([uav_pos[0], uav_pos[1]])
        self.goal_marker.set_offsets([goal_pos[0], goal_pos[1]])
        
        # Add rotation indicator
        if label_str in ['rotate_left', 'rotate_right']:
            self.ax_path.text(
                uav_pos[0], uav_pos[1], '↻' if label_str == 'rotate_left' else '↺',
                color='purple', fontsize=12, ha='center', va='center'
            )
        
        # Update display axes limits
        all_x = self.path_x + [goal_pos[0]]
        all_y = self.path_y + [goal_pos[1]]
        if all_x and all_y:
            self.ax_path.set_xlim(min(all_x) - 1, max(all_x) + 1)
            self.ax_path.set_ylim(min(all_y) - 1, max(all_y) + 1)

        self.fig_path.canvas.draw_idle()
        self.fig_path.canvas.flush_events()

    def close(self):
        plt.ioff()

        plt.close('all')
        
