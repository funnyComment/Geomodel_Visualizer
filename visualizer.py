from utils import load_geomodel, compute_robot_pose, draw_robot
import pandas as pd
import os
import argparse

# Parse the command-line arguments
parser = argparse.ArgumentParser(description="Simple DH parameter visualizer from a .geo file")
parser.add_argument("file", type=str,nargs='?', help="Execute: >python vilualizer.py 'Path to the .geo XML file'")
args = parser.parse_args()

def main():
    """
    Loads a robot model from a .geo file, extracts link parameters, computes DH parameters, 
    calculates transformation matrices (poses), and visualizes the robot.
    """
    # Select filepath
    foldername = 'models'
    filename = 'R2000iB_200R.geo'
    filepath = os.path.join(foldername, filename)

    # Check if the 'file' argument was provided
    if not args.file:
        print("Warnnig: No file path provided. Using default path\n")
    else:
        file_path = args.file
        print(f"Processing file: {file_path}")
        filepath = file_path

    # Load robot data
    robot_data = load_geomodel(filepath)
    if not robot_data:
        print("Robot model could not be loaded. Exiting.")
        return

    # Display model features
    print('\n-- Robot Model --')
    print('RCS version = ',robot_data['RcsVersion'])
    print(f"{robot_data['Manufacturer']}, Controller: {robot_data['SupportedController']}")
    print(f"Model: {robot_data['ManipulatorName']}")

    # Create and display a DataFrame with the link parameters
    df = pd.DataFrame.from_dict(robot_data['Links'], orient='index')
    print("\nLink parameter table:")
    print(df)

    # Extract and organize DH parameters 
    dh_params = [(param['A'], param['r'], param['d'], param['T']) for param in robot_data['Links'].values()]

    # Compute robot poses
    poses = compute_robot_pose(dh_params)

    # Visualize the robot
    draw_robot(poses)

if __name__ == '__main__':
    main()
