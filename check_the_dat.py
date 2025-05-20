import pickle
import numpy as np
from collections import Counter

with open("uav_stream_data.pkl", "rb") as f:
    frame_data = pickle.load(f)
    depth = frame_data["depth"]

    print("Original depth shape:", np.array(depth).shape)

    labels = frame_data['label']

    total = len(labels)
    print(f"📦 Total labels: {total}\n")

    for i in range(total):
        print(f"🧾 Entry {i}:")
        for key in frame_data:
            value = frame_data[key]
            if hasattr(value, 'shape'):
                print(f" 1 {key}: ndarray shape {value.shape}")
            else:
                print(f" 2 {key}: {value}")
        print()

