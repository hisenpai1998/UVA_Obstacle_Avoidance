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

from coppeliasim_zmqremoteapi_client    import RemoteAPIClient
from config                             import OBJECT_PATHS

class CoppeliaSimInterface:

    def __init__(self, host='localhost', port=23000):
        """
        Interface for CoppeliaSim using ZMQ Remote API.
        
        """
        try:
            self.client = RemoteAPIClient(host, port)
            self.sim = self.client.getObject('sim')
            print("Successfully connected to CoppeliaSim")
            self.object_paths = OBJECT_PATHS
            self.handles = {}
            for key, paths in self.object_paths.items():
                if isinstance(paths, list):
                    self.handles[key] = []
                    for p in paths:
                        try:
                            handle = self.sim.getObject(p)
                            self.handles[key].append(handle)
                            print(f"Successfully got handle for {p}")
                        except Exception as e:
                            print(f"Failed to get handle for {p}: {e}")
                else:
                    try:
                        handle = self.sim.getObject(paths)
                        self.handles[key] = handle
                        print(f"Successfully got handle for {paths}")
                    except Exception as e:
                        print(f"Failed to get handle for {paths}: {e}")
        except Exception as e:
            print(f"[ERROR] Failed to initialize CoppeliaSimInterface: {e}")
            exit()


    def get_object_handles(self):
        """
        Retrieve object handles from the simulator.
        
        """
        try:
            for key, path in self.object_paths.items():

                if isinstance(path, str):
                    print(f"Trying to get handle for {key} at path: {path}")
                    self.handles[key] = self.sim.getObject(path)
                elif isinstance(path, list):
                    print(f"Trying to get handles for {key} at paths: {path}")
                    self.handles[key] = []
                    for single_path in path:
                        if not isinstance(single_path, str):
                            raise TypeError(f"Object path in list for {key} must be a string, got {type(single_path)}")
                        self.handles[key].append(self.sim.getObject(single_path))
                else:
                    raise TypeError(f"Object path for {key} must be a string or list of strings, got {type(path)}")

            # Enable explicit handling for the vision sensor
            self.sim.setExplicitHandling(self.handles['vision_sensor'], 1)

        except Exception as e:
            print(f"[ERROR] Failed to access simulation objects: {e}")
            print(f"Current object paths: {self.object_paths}")  # Debug print
            self.stop_simulation()
            exit()


    def start_simulation(self):
        """
        Start the simulation and enable stepping.

        """
        self.sim.startSimulation()
        print("Simulation started.")
        self.sim.setStepping(True)



    def stop_simulation(self):
        """
        Stop the simulation.

        """
        self.sim.stopSimulation()
        print("Simulation stopped.")



    def step(self):
        """
        Advance the simulation by one step (when stepping is enabled).

        """
        self.sim.step()



    def get_simulation_time(self):
        """
        Get the current simulation time.

        """
        return self.sim.getSimulationTime()



    def get_vision_sensor_data(self):
        """
        Get depth image data from the vision sensor.    

        """
        self.sim.handleVisionSensor(self.handles['vision_sensor'])
        depth_bytes, resolution = self.sim.getVisionSensorDepth(self.handles['vision_sensor'])
        return depth_bytes, resolution

    

    def set_object_position(self, object_key, position, index=None):
        """
        Set the position of a specified object.
        
        Args:
            object_key (str): Key of the object in handles dictionary
            position (list): [x, y, z] position coordinates
            index (int, optional): Index for objects with multiple instances. Defaults to None.
        """
        handle = self.handles[object_key]
        if isinstance(handle, list):
            if index is None:
                raise ValueError(f"Index required for object '{object_key}' with multiple instances")
            self.sim.setObjectPosition(handle[index], -1, position)
        else:
            self.sim.setObjectPosition(handle, -1, position)



    def get_object_position(self, object_key, index=None):
        """
        Get the position of a specified object.

        Args:
            object_key (str): Key of the object in handles dictionary
            index (int, optional): Index for objects with multiple instances. Defaults to None.
        
        Returns:
            list: [x, y, z] position coordinates
        """
        handle = self.handles[object_key]
        if isinstance(handle, list):
            if index is None:
                raise ValueError(f"Index required for object '{object_key}' with multiple instances")
            return self.sim.getObjectPosition(handle[index], -1)
        return self.sim.getObjectPosition(handle, -1)

