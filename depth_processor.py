import numpy        as np
from scipy.ndimage  import zoom
from config         import DEPTH_THRESHOLD

class DepthAnalyzer:
    def __init__(self, depth_threshold):
        """
        Initialize the depth analyzer.
        
        """
        self.depth_threshold = depth_threshold

    def process_depth_data(self, depth_bytes, resolution):
        """
        Convert raw depth bytes to a resized depth array.
        
        """
        resX, resY      = resolution
        depth_array     = np.frombuffer(depth_bytes, dtype=np.float32).reshape(resY, resX)
        depth_array     = np.flipud(depth_array)
        depth_resized   = zoom(depth_array, (256 / resY, 256 / resX), order=1)
        return depth_resized

    def analyze_9_regions(self, depth_image):
        """
        Analyze the depth image by focusing on 5 key regions.
        
        """
        h, w          = depth_image.shape
        region_depths = {}

        # Divide into 3x3 regions, but only process front, up, down, left, right
        h_third = h // 3
        w_third = w // 3

        regions = {
            'up'    : depth_image[         :h_third,    w_third:2*w_third],
            'left'  : depth_image[h_third:2*h_third,             :w_third],
            'front' : depth_image[h_third:2*h_third,    w_third:2*w_third],
            'right' : depth_image[h_third:2*h_third,           2*w_third:],
            'down'  : depth_image[       2*h_third:,    w_third:2*w_third]
        }

        print("\n🟦 Terminal View: 5 Key Region Depths")
        label_order = [
            ['up'],
            ['left', 'front', 'right'],
            ['down']
        ]

        # Compute avg and store
        for label, region in regions.items():
            valid   = region[region > DEPTH_THRESHOLD]
            avg     = np.mean(valid) if valid.size > 0 else np.mean(region)
            region_depths[label] = avg

        # Print 5-region layout
        print("\n🟫 Depth Region Average Matrix:")
        for row in label_order:
            row_str = "  ".join(f"{region_depths.get(label, 0):>6.2f}" for label in row)
            print(row_str)

        # Identify safe regions
        safe_regions = [region for region, depth in region_depths.items() if depth > self.depth_threshold]

        return safe_regions, region_depths

    def choose_direction(self, safe_regions, region_depths, safe_sectors=None, sectors=None):
        """
        Choose the safest direction based on the 5 regions.
        
        """
        # Direction map with labels and normalized 3D vectors
        direction_map = {
            'front':    {'label': 0, 'label_str': 'forward',    'vector': np.array([ 0, 1,  0])},
            'back':     {'label': 1, 'label_str': 'backward',   'vector': np.array([ 0, -1, 0])},
            'up':       {'label': 2, 'label_str': 'up',         'vector': np.array([ 0, 0,  1])},
            'down':     {'label': 3, 'label_str': 'down',       'vector': np.array([ 0, 0, -1])},
            'left':     {'label': 4, 'label_str': 'left',       'vector': np.array([-1, 0,  0])},
            'right':    {'label': 5, 'label_str': 'right',      'vector': np.array([ 1, 0,  0])},
            'stop':     {'label': 6, 'label_str': 'stop',       'vector': np.array([ 0, 0,  0])},
            'rotate_left':  {'label': 7, 'label_str': 'rotate_left',  'vector': np.array([0, 0, 0])},
            'rotate_right': {'label': 8, 'label_str': 'rotate_right', 'vector': np.array([0, 0, 0])}
        }

        if region_depths.get('front', float('inf')) < self.depth_threshold * 1.5:
            print("Emergency stop - obstacle too close in front!")
            return 6, 'stop', np.array([0, 0, 0])
    
        if not safe_regions:
            print("No safe regions detected - stopping!")
            return 6, 'stop', np.array([0, 0, 0])

        best_region = max(safe_regions, key=lambda region: region_depths[region])
        best_depth  = region_depths[best_region]
        print(f"Best region: {best_region}, Depth: {best_depth}")

        if best_depth < self.depth_threshold * 2:
            print("All regions too close - stopping!")
            return 6, 'stop', np.array([0, 0, 0])

        direction = direction_map.get(best_region, {'label': 6, 'label_str': 'stop', 'vector': np.array([0, 0, 0])})
        return direction['label'], direction['label_str'], direction['vector']
        

    def vector_to_label(self, vector, yaw):
        """
        Convert a 3D vector (world frame) into a label based on local drone yaw.

        """
        if np.linalg.norm(vector) < 1e-6: 
            return 6, 'stop', vector

        # Rotate vector into drone's local frame (reverse yaw)
        cos_yaw = np.cos(-yaw)
        sin_yaw = np.sin(-yaw)

        rot_matrix = np.array([
            [cos_yaw, -sin_yaw, 0],
            [sin_yaw,  cos_yaw, 0],
            [     0 ,       0, 1]
        ])

        local_vector = rot_matrix @ (vector / np.linalg.norm(vector))

        direction_map = {
            'front':    {'label': 0, 'label_str': 'forward',    'vector': np.array([ 0, 1,  0])},
            'back':     {'label': 1, 'label_str': 'backward',   'vector': np.array([ 0, -1, 0])},
            'up':       {'label': 2, 'label_str': 'up',         'vector': np.array([ 0, 0,  1])},
            'down':     {'label': 3, 'label_str': 'down',       'vector': np.array([ 0, 0, -1])},
            'left':     {'label': 4, 'label_str': 'left',       'vector': np.array([-1, 0,  0])},
            'right':    {'label': 5, 'label_str': 'right',      'vector': np.array([ 1, 0,  0])},
            'stop':     {'label': 6, 'label_str': 'stop',       'vector': np.array([ 0, 0,  0])},
            'rotate_left':  {'label': 7, 'label_str': 'rotate_left',  'vector': np.array([0, 0, 0])},
            'rotate_right': {'label': 8, 'label_str': 'rotate_right', 'vector': np.array([0, 0, 0])}
        }

        max_dot = -1
        best_label = None
        for key, value in direction_map.items():
            dot = np.dot(local_vector, value['vector'])
            if dot > max_dot:
                max_dot = dot
                best_label = key

        if max_dot < 0.5:
            return 6, 'stop', vector
        return direction_map[best_label]['label'], direction_map[best_label]['label_str'], vector
