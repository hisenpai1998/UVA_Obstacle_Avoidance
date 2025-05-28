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

import pickle

class DataStreamer:
    def __init__(self, file_path):
        """
        Initialize the data stream file.
        """
        self.file = open(file_path, "ab")

    def stream_frame(self, frame_data):
        """
        Save a frame of simulation data.
        """
        pickle.dump(frame_data, self.file)

    def close(self):
        """
        Close the data file.
        """
        if self.file is not None:
            self.file.close()

    def read_saved_data(self, file_path):
        """
        Reads and prints data from the saved UAV stream file.
        """
        with open(file_path, "rb") as f:
            print("Reading saved UAV data...")
        try:
            while True:
                frame_data = pickle.load(f)
                print("Label:", frame_data["label"])
                print("UAV Position:", frame_data["drone_position"])
                print("Goal Position:", frame_data["goal_position"])
                print("Direction Vector:", frame_data["direction_vector"])
                print("Depth Sample (top-left 3x3):")
                print(frame_data["depth"][:3, :3])
                print("---")
        except EOFError:   
                print("Finished reading all frames.")