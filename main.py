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