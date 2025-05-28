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

import random
import numpy as np

class EnvironmentGenerator:
    def __init__(self, sim_interface, obstacle_manager, floor_handle, target_handle, drone_handle, goal_handle):
        self.sim_interface = sim_interface
        self.obstacle_manager = obstacle_manager
        self.floor_handle = floor_handle
        self.target_handle = target_handle
        self.drone_handle = drone_handle
        self.goal_handle = goal_handle
        self.fx, self.fy, self.fz = sim_interface.get_object_position('floor')
        self.UAV_SPAWN_HEIGHT = 1.0
        self.EDGE_INSET = 0.2
        self.MIN_CLEARANCE = 3.0

    def get_floor_extents(self):
        min_x = self.sim_interface.sim.getObjectFloatParameter(self.floor_handle, 15)[1]
        min_y = self.sim_interface.sim.getObjectFloatParameter(self.floor_handle, 16)[1]
        max_x = self.sim_interface.sim.getObjectFloatParameter(self.floor_handle, 18)[1]
        max_y = self.sim_interface.sim.getObjectFloatParameter(self.floor_handle, 19)[1]
        return min_x, max_x, min_y, max_y

    def clear_environment(self):
        for h in self.sim_interface.sim.getObjectsInTree(self.sim_interface.sim.handle_scene, self.sim_interface.sim.handle_all, 1):
            try:
                name = self.sim_interface.sim.getObjectAlias(h, 5)
                if name.startswith(("TreeTrunk", "TreeBranch", "dynamic_obs")):
                    self.sim_interface.sim.removeObject(h)
            except:
                pass

    def create_box(self, name, pos, size, orientation=None, is_static=True):
        h = self.sim_interface.sim.createPrimitiveShape(self.sim_interface.sim.primitiveshape_cuboid, size, 0)
        self.sim_interface.sim.setObjectInt32Param(h, self.sim_interface.sim.shapeintparam_static, int(is_static))
        self.sim_interface.sim.setObjectInt32Param(h, self.sim_interface.sim.shapeintparam_respondable, 1)
        self.sim_interface.sim.setObjectAlias(h, name, True)
        self.sim_interface.sim.setObjectPosition(h, -1, pos)
        if orientation is not None:
            self.sim_interface.sim.setObjectOrientation(h, -1, orientation)
        return h


    def point_camera_toward_goal(self):
        """
        Rotate the vision sensor to face the goal from UAV position.
        Only uses yaw rotation to keep the UAV level while pointing toward the goal.
        """
        target_pos = np.array(self.sim_interface.get_object_position('target'))
        goal_pos = np.array(self.sim_interface.get_object_position('goal'))
        direction = goal_pos - target_pos
        
        # Calculate horizontal angle (yaw) and invert it for correct orientation
        yaw = np.arctan2(direction[1], direction[0]) + np.pi  # Add pi to invert the direction
        
        # CoppeliaSim expects orientation in Euler angles [roll, pitch, yaw]
        # We keep roll and pitch at 0, only use yaw for horizontal rotation
        self.sim_interface.sim.setObjectOrientation(
            self.sim_interface.handles['drone'], -1, [0.0, 0.0, yaw]
        )

    def spawn_uav_and_goal(self):
        min_x, max_x, min_y, max_y = self.get_floor_extents()
        x_min, x_max = min_x + self.EDGE_INSET, max_x - self.EDGE_INSET
        y_min, y_max = min_y + self.EDGE_INSET, max_y - self.EDGE_INSET
        base_z = self.fz + self.UAV_SPAWN_HEIGHT

        axis = random.choice(['x', 'y'])
        side = random.choice(['min', 'max'])
        if axis == 'x':
            ux = x_min if side == 'min' else x_max
            uy = random.uniform(y_min, y_max)
            gx = x_max if side == 'min' else x_min
            gy = random.uniform(y_min, y_max)
        else:
            uy = y_min if side == 'min' else y_max
            ux = random.uniform(x_min, x_max)
            gy = y_max if side == 'min' else y_min
            gx = random.uniform(x_min, x_max)

        uav_pos = [ux, uy, base_z]
        goal_pos = [gx, gy, base_z]

        self.sim_interface.set_object_position('drone', uav_pos)
        self.sim_interface.set_object_position('target', uav_pos)
        self.sim_interface.set_object_position('goal', goal_pos)
        
        # Set initial UAV orientation to face the goal with inverted direction
        direction = np.array(goal_pos) - np.array(uav_pos)
        yaw = np.arctan2(direction[1], direction[0]) + np.pi  # Add pi to invert the direction
        self.sim_interface.sim.setObjectOrientation(
            self.sim_interface.handles['drone'], -1, [0.0, 0.0, yaw]
        )

        print(f" UAV/Target at {uav_pos} on {axis}-edge ({side}) inset")
        print(f" Goal at {goal_pos} on opposite {axis}-edge")

        return uav_pos, goal_pos

    def spawn_trees(self, num_trees, exclude_positions):
        min_x, max_x, min_y, max_y = self.get_floor_extents()
        for i in range(num_trees):
            for _ in range(30):
                th = random.uniform(4.0, 6.0)  # Taller trunk
                sx, sy = random.uniform(0.2, 0.4), random.uniform(0.2, 0.4)  # Thicker trunk
                x = random.uniform(min_x, max_x)
                y = random.uniform(min_y, max_y)

                if all(np.linalg.norm(np.array([x, y]) - np.array(p[:2])) > self.MIN_CLEARANCE for p in exclude_positions):
                    zc = self.fz + th / 2
                    trunk_h = self.create_box(f"TreeTrunk[{i}]", [x, y, zc], [sx, sy, th])
                    self.obstacle_manager.add_obstacle(trunk_h, is_dynamic=False)

                    for j in range(random.randint(2, 4)):
                        bl = random.uniform(1.0, 2.0)
                        bh = random.uniform(th * 0.4, th * 0.95)
                        bzc = self.fz + bh
                        yaw = random.uniform(0, 2 * np.pi)
                        od = sx / 2 + bl / 2
                        dx, dy = od * np.cos(yaw), od * np.sin(yaw)
                        branch_h = self.create_box(
                            f"TreeBranch[{i}_{j}]",
                            [x + dx, y + dy, bzc],
                            [bl, 0.1, 0.1],
                            orientation=[0, 0, yaw]
                        )
                        self.obstacle_manager.add_obstacle(branch_h, is_dynamic=False)
                    break
            else:
                print(f"⚠️ Could not place tree {i}")

    def spawn_dynamic_obstacles(self, n_dyn, dyn_size, vel_xyz, exclude_positions):
        min_x, max_x, min_y, max_y = self.get_floor_extents()
        for i in range(n_dyn):
            for _ in range(30):
                x = random.uniform(min_x, max_x)
                y = random.uniform(min_y, max_y)
                z = random.uniform(self.fz + 0.3, self.fz + 2.0)
                if all(np.linalg.norm(np.array([x, y]) - np.array(p[:2])) > self.MIN_CLEARANCE for p in exclude_positions):
                    h = self.sim_interface.sim.createPrimitiveShape(self.sim_interface.sim.primitiveshape_spheroid, dyn_size, 0)
                    self.sim_interface.sim.setObjectInt32Param(h, self.sim_interface.sim.shapeintparam_static, 0)
                    self.sim_interface.sim.setObjectInt32Param(h, self.sim_interface.sim.shapeintparam_respondable, 1)
                    self.sim_interface.sim.setObjectAlias(h, f"dynamic_obs_{i}", True)
                    self.sim_interface.sim.setObjectPosition(h, -1, [x, y, z])
                    v = np.array(vel_xyz) * np.random.choice([-1, 1], 3) * random.uniform(0.5, 1.2)
                    self.obstacle_manager.add_obstacle(h, is_dynamic=True, velocity=v, size=dyn_size)
                    break
            else:
                print(f"⚠️ Could not place dynamic obstacle {i}")

    def setup_environment(self, tree_count, n_dyn, dyn_size, vel_xyz):
        self.clear_environment()
        uav_pos, goal_pos = self.spawn_uav_and_goal()
        self.spawn_trees(tree_count, [uav_pos, goal_pos])
        self.spawn_dynamic_obstacles(n_dyn, dyn_size, vel_xyz, [uav_pos, goal_pos])