import numpy as np

class ObstacleManager:
    def __init__(self, sim_interface, obstacle_keys):

        """Initialize the obstacle manager."""

        self.sim_interface      = sim_interface
        self.num_obstacles      = len(obstacle_keys)    # Store the number of obstacles
        self.virtual_obstacles  = []                    # Store virtual obstacle positions
        self.time               = 0

    def update_obstacles(self, dt):

        """Update the positions of all obstacles."""

        self.time += dt
        for i in range(self.num_obstacles):
            try:
                key = self.obstacle_keys[i]
                # Example: Simple sinusoidal motion for obstacles
                current_pos = self.sim_interface.get_object_position('obstacles', index=i)
                x = current_pos[0] + 0.1 * np.sin(self.time + i * np.pi / 3)                    # Unique motion per Cuboid
                y = current_pos[1] + 0.1 * np.cos(self.time + i * np.pi / 3)                    # Unique motion per Cuboid      
                z = current_pos[2]  # Keep z constant
                self.sim_interface.set_object_position('obstacles', [x, y, z], index=i)

            except Exception as e:
                print(f"[ERROR] Failed to update obstacle {i}: {e}")
                continue

    def add_virtual_obstacles(self, virtual_obstacles):

        """Add virtual obstacles to the manager."""

        self.virtual_obstacles = virtual_obstacles

    def get_obstacle_positions(self):

        """Get the current positions of all obstacles, including virtual ones."""
        
        # Get positions of physical obstacles
        dynamic_positions = {}
        for i in range(self.num_obstacles):
            try:
                dynamic_positions[f"obstacle_{i}"] = self.sim_interface.get_object_position('obstacles', index=i)
            except Exception as e:
                print(f"[ERROR] Failed to get position for obstacle {i}: {e}")
                continue

        # Add virtual obstacles
        virtual_positions = {f"virtual_{i}": pos for i, pos in enumerate(self.virtual_obstacles)}
        
        return {**dynamic_positions, **virtual_positions}