import numpy        as np
from scipy.ndimage  import zoom
from config         import DEPTH_THRESHOLD

class DepthAnalyzer:
    def __init__(self, depth_threshold):

        """Initialize the depth analyzer."""

        self.depth_threshold = depth_threshold

    def process_depth_data(self, depth_bytes, resolution):

        """Convert raw depth bytes to a resized depth array."""

        resX, resY      = resolution
        depth_array     = np.frombuffer(depth_bytes, dtype=np.float32).reshape(resY, resX)
        depth_array     = np.flipud(depth_array)
        depth_resized   = zoom(depth_array, (256 / resY, 256 / resX), order=1)
        return depth_resized

    def analyze_9_regions(self, depth_image):

        """Analyze the depth image by dividing it into 9 regions."""

        h, w    = depth_image.shape
        region_depths = {}

        # Divide into 3x3 regions
        h_third = h // 3
        w_third = w // 3

        regions = {
            'up-left'    : depth_image[:h_third, :w_third],
            'up'         : depth_image[:h_third, w_third:2*w_third],
            'up-right'   : depth_image[:h_third, 2*w_third:],
            'left'       : depth_image[h_third:2*h_third, :w_third],
            'front'      : depth_image[h_third:2*h_third, w_third:2*w_third],
            'right'      : depth_image[h_third:2*h_third, 2*w_third:],
            'down-left'  : depth_image[2*h_third:, :w_third],
            'down'       : depth_image[2*h_third:, w_third:2*w_third],
            'down-right' : depth_image[2*h_third:, 2*w_third:] 
        }

        print("\n🟦 Terminal View: 3×3 Region Depths")
        label_order = [
            ['up-left'  , 'up'      , 'up-right'  ],
            ['left'     , 'front'   , 'right'     ],
            ['down-left', 'down'    , 'down-right']
        ]

        # Compute avg and store
        for label, region in regions.items():
            valid = region[region > DEPTH_THRESHOLD]
            avg = np.mean(valid) if valid.size > 0 else np.mean(region)
            region_depths[label] = avg

        #  Print 3x3 grid of average values
        print("\n🟫 Depth Region Average Matrix (3x3):")
        for row in label_order:
            row_str = "  ".join(f"{region_depths[label]:>6.2f}" for label in row)
            print(row_str)

        # Identify safe regions
        safe_regions = [region for region, depth in region_depths.items() if depth > self.depth_threshold]

        return safe_regions, region_depths

    def find_safe_directions(self, depth_image):

        """Analyze depth image to find safe directions in 2D (16 SECTOR)."""

        h, w                = depth_image.shape
        center_y, center_x  = h // 2, w // 2
        Y, X                = np.ogrid[:h, :w]
        angles              = np.arctan2(center_y - Y, X - center_x) % (2 * np.pi)
        dist                = np.sqrt((X - center_x)**2 + (Y - center_y)**2)
        max_radius          = min(center_x, center_y)

        sectors         = []
        sector_angles   = []
        num_sectors     = 16
        angle_step      = 2 * np.pi / num_sectors

        for i in range(num_sectors):
            start   = i * angle_step
            end     = (start + angle_step) % (2 * np.pi)
            if start < end:
                mask = (start <= angles) & (angles < end)
            else:
                mask = (start <= angles) | (angles < end)

            mask            &= (dist < max_radius)
            sector_data     = depth_image[mask]
            valid           = sector_data[sector_data > 0.05]
            avg_depth       = np.mean(valid) if valid.size > 0 else 0
            sectors.append(avg_depth)
            sector_angles.append(start + angle_step / 2)

        # Identify safe regions
        safe_sectors = [i for i, d in enumerate(sectors) if d > self.depth_threshold]
        return safe_sectors, sector_angles, sectors

    def choose_direction(self, safe_regions, region_depths, safe_sectors=None, sectors=None):

        """Choose the safest direction based on the 9 regions."""

        if region_depths['front'] < self.depth_threshold * 1.5:  # Adding safety margin
            print("⚠️ Emergency stop - obstacle too close in front!")
            return "stop", 0
    
        # If no safe regions, stop
        if not safe_regions:
            print("⚠️ No safe regions detected - stopping!")
            return "stop", 0

        # Choose the region with the greatest depth
        best_region     = max(safe_regions, key=lambda region: region_depths[region])
        
        # Show the best region and its depth    
        best_depth      = region_depths[best_region]  # Uncommented this line
        print(f"📦Best region: {best_region}, Depth: {best_depth}")


        # Map regions to directions and angles (simplified)
        direction_map = {
            'front'     : ('forward',   0),            # 0   degrees
            'up'        : ('up'     ,   0),            # Treated as forward for simplicity
            'down'      : ('down'   , 180),            # Treated as downward
            'left'      : ('left'   , 270),            # 270 degrees
            'right'     : ('right'  ,  90),            # 90  degrees
            'up-left'   : ('left'   , 270),
            'down-left' : ('left'   , 270),
            'up-right'  : ('right'  ,  90),
            'down-right': ('right'  ,  90)
        }

        # Add minimum depth threshold for movement
        if best_depth < self.depth_threshold * 2:  # Double the threshold for safety
            print("⚠️ All regions too close - stopping!")
            return "stop", 0

        # Default to stop if the region isn't in the map
        label, angle = direction_map.get(best_region, ('stop', 0))
        return label, angle  # Returning sector as 0 since we're not using sectors

    def map_region_to_angle(self, region):
        """Map a region to an angle in radians for virtual obstacle placement."""
        angle_map = {
                'up-left':      [-0.5, 0.5, 0],
                'up':           [   0, 0.5, 0],
                'up-right':     [ 0.5, 0.5, 0],
                'left':         [-0.5,   0, 0],
                'front':        [   0,   0, 0],  # Center, might not need virtual obstacle
                'right':        [ 0.5,   0, 0],
                'down-left':    [-0.5,-0.5, 0],
                'down':         [   0,-0.5, 0],
                'down-right':   [ 0.5,-0.5, 0]
        }
        return angle_map.get(region, 0)  # Default to 0 if region not found
    