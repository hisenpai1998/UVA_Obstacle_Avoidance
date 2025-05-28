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


from config                 import STEP_SIZE, DEPTH_THRESHOLD, SIMULATION_TIME, STREAM_FILE
from config                 import OBJECT_PATHS

from coppeliasim_interface  import CoppeliaSimInterface
from keyboard_manager       import KeyboardController
from depth_processor        import DepthAnalyzer
from plot                   import Visualizer
from stream_manager         import DataStreamer
from environment_generator  import EnvironmentGenerator
from UAV_navigator          import UAVNavigator
from obstacle_manager       import ObstacleManager
from APF_navigator          import APFNavigator
from collision_detector     import CollisionDetector

class UAVSimulator:

    def __init__(self, host='localhost', port=23000):
        """
        Initialize the UAV simulator with the given host and port.
        """
        self.sim_interface       = CoppeliaSimInterface(host, port)
        self.keyboard_controller = KeyboardController()
        self.depth_analyzer      = DepthAnalyzer(DEPTH_THRESHOLD)
        self.visualizer          = Visualizer()
        self.data_streamer       = DataStreamer(STREAM_FILE)
        self.obstacle_manager    = ObstacleManager(self.sim_interface, OBJECT_PATHS.get('walls', []))
        self.apf_navigator       = APFNavigator()
        self.collision_detector  = CollisionDetector()
        self.environment_generator = EnvironmentGenerator(
            self.sim_interface,
            self.obstacle_manager,
            self.sim_interface.handles['floor'],
            self.sim_interface.handles['target'],
            self.sim_interface.handles['drone'],
            self.sim_interface.handles['goal']
        )

        self.sim_interface.start_simulation()
        self.sim_interface.get_object_handles()
        self.keyboard_controller.start()

        self.navigator = UAVNavigator(
            self.sim_interface,
            self.keyboard_controller,
            self.depth_analyzer,
            self.visualizer,
            self.data_streamer,
            self.obstacle_manager,
            self.apf_navigator,
            self.collision_detector,
            SIMULATION_TIME,
            STEP_SIZE
        )

    def run(self):
        """
        Run the UAV simulation.
        """
        self.environment_generator.setup_environment(
            tree_count=60,
            n_dyn=5,
            dyn_size=[0.2, 0.2, 0.2],
            vel_xyz=[0.1, 0.1, 0.15]
        )
        self.navigator.current_pos = self.sim_interface.get_object_position('target')
        self.navigator.run()

    def read_saved_data(self):
        """
        Read the saved data from the simulation.
        """ 
        self.data_streamer.read_saved_data(STREAM_FILE)