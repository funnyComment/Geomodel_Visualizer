from utils import load_geomodel, compute_robot_pose, draw_robot
import pandas as pd
import os

def main():
    """
    Loads a robot model from a .geo file, extracts link parameters, computes DH parameters, 
    calculates transformation matrices (poses), and visualizes the robot.
    """
    # Select filepath
    foldername = 'models'
    filename = 'R2000iB_200R.geo'
    filepath = os.path.join(foldername, filename)

    # Load robot data
    robot_data = load_geomodel(filepath)
    if not robot_data:
        print("Robot model could not be loaded. Exiting.")
        return

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
