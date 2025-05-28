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

from UAV_simulator import UAVSimulator

def main():

    """
    Run the UAV simulation.
    
    """
    simulator = UAVSimulator()
    simulator.run()
    simulator.read_saved_data()

if __name__ == "__main__":
    main()