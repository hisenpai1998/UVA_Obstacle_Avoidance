from config                 import STEP_SIZE, DEPTH_THRESHOLD, SIMULATION_TIME, STREAM_FILE
from config                 import OBJECT_PATHS


from coppeliasim_interface  import CoppeliaSimInterface
from keyboard_manager       import KeyboardController
from depth_processor        import DepthAnalyzer
from plot                   import Visualizer
from stream_manager         import DataStreamer


from UAV_navigator          import UAVNavigator
from obstacle_manager       import ObstacleManager
from APF_navigator          import APFNavigator
from collision_detector     import CollisionDetector


class UAVSimulator:
    def __init__(self, host='localhost', port=23000):
        """Initialize the UAV simulator."""
        self.sim_interface       = CoppeliaSimInterface(host, port)
        self.keyboard_controller = KeyboardController()
        self.depth_analyzer      = DepthAnalyzer(DEPTH_THRESHOLD)
        self.visualizer          = Visualizer()
        self.data_streamer       = DataStreamer(STREAM_FILE)
        self.obstacle_manager    = ObstacleManager(self.sim_interface, OBJECT_PATHS['obstacles'])
        self.apf_navigator       = APFNavigator()
        self.collision_detector  = CollisionDetector()

        # Start the simulation and components
        self.sim_interface.start_simulation()
        self.sim_interface.get_object_handles()
        self.keyboard_controller.start()

        # Initialize the navigator
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
        """Run the simulation."""
        self.navigator.run()

    def read_saved_data(self):
        """Read and print the saved data."""
        self.data_streamer.read_saved_data(STREAM_FILE)