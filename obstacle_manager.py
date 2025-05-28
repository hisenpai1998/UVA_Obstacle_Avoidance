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

import numpy as np

class ObstacleManager:
    def __init__(self, sim_interface, static_paths=[]):
        """
        Initialize the obstacle manager.
        """	
        self.sim_interface = sim_interface
        self.floor_handle = sim_interface.handles['floor']
        self.fx, self.fy, self.fz = sim_interface.sim.getObjectPosition(self.floor_handle, -1)
        self.min_x, self.max_x, self.min_y, self.max_y = self.get_floor_extents()
        self.min_z = self.fz + 0.3
        self.max_z = self.fz + 2.0
        self.bounds = [[self.min_x, self.max_x], [self.min_y, self.max_y], [self.min_z, self.max_z]]
        self.static_obstacles = [sim_interface.sim.getObject(p) for p in static_paths]
        self.dynamic_obstacles = []

    def add_obstacle(self, handle, is_dynamic=False, velocity=None, size=None):
        """
        Add an obstacle to the manager.
        """
        if is_dynamic:
            self.dynamic_obstacles.append({'handle': handle, 'velocity': np.array(velocity), 'size': size})
        else:
            self.static_obstacles.append(handle)

    def update_obstacles(self, dt):
        """
        Update the positions of dynamic obstacles.
        """
        for obs in self.dynamic_obstacles:
            pos = np.array(self.sim_interface.sim.getObjectPosition(obs['handle'], -1))
            vel = obs['velocity']
            pos += vel * dt
            s = obs['size']
            for j in range(3):
                lo, hi = self.bounds[j]
                half = s[j] / 2
                if pos[j] - half < lo:
                    pos[j] = lo + half
                    vel[j] *= -0.8
                elif pos[j] + half > hi:
                    pos[j] = hi - half
                    vel[j] *= -0.8
            self.sim_interface.sim.setObjectPosition(obs['handle'], -1, pos.tolist())
            obs['velocity'] = vel

    def get_obstacle_positions(self):
        """
        Get the positions of all obstacles.
        """
        positions = {}
        for h in self.static_obstacles:
            positions[h] = self.sim_interface.sim.getObjectPosition(h, -1)
        for obs in self.dynamic_obstacles:
            positions[obs['handle']] = self.sim_interface.sim.getObjectPosition(obs['handle'], -1)
        return positions

    def get_floor_extents(self):
        """
        Get the extents of the floor.
        """
        min_x = self.sim_interface.sim.getObjectFloatParameter(self.floor_handle, 15)[1]
        min_y = self.sim_interface.sim.getObjectFloatParameter(self.floor_handle, 16)[1]
        max_x = self.sim_interface.sim.getObjectFloatParameter(self.floor_handle, 18)[1]
        max_y = self.sim_interface.sim.getObjectFloatParameter(self.floor_handle, 19)[1]
        return min_x, max_x, min_y, max_y