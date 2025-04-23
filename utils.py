import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import xml.etree.ElementTree as ET
import json
import os


def dh_transform_matrix(alpha, d, a, theta):
    """Calculate the homogeneous transformation matrix according to the DH parameters"""
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
        [0, 0, 0, 1]
    ])

def compute_robot_pose(dh_params):
    """
    Receives a list of DH parameters (alpha, a, d, theta) and returns the accumulated 
    transformation matrices for each link relative to the base reference frame.
    """
    T = np.eye(4)  # Identity matrix as initial reference
    poses = [T]  # List to store the accumulated poses

    for alpha, a, d, theta in dh_params:
        T = T @ dh_transform_matrix(alpha, a, d, theta)  # Cumulative multiplication
        poses.append(T)
    
    return poses

def draw_robot(poses, ax=None):
    """
    Draws the position and orientation of each link in 3D space.
    param poses: List of 4x4 homogeneous transformation matrices.
    """
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
    
    # Setting chart limits
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([0, 2])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Robot kinematics")
    
    # Draw each link
    for i in range(len(poses) - 1):
        p0 = poses[i][:3, 3]  # Link position i
        p1 = poses[i + 1][:3, 3]  # Link position i+1
        
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], 'bo-', linewidth=2, markersize=5)
        
        # Dibujar ejes de orientación
        R = poses[i][:3, :3]  # Rotation matrix
        axis_length = 0.2
        for j, color in enumerate(['r', 'g', 'b']):  # X=red, Y=green, Z=blue
            ax.quiver(p0[0], p0[1], p0[2], R[0, j], R[1, j], R[2, j],
                      color=color, length=axis_length)
    
    plt.show()

def load_geomodel(filepath='models/default.geo'):

    """
    Parses a .geo XML file describing a robotic manipulator model and returns its structured data as a dictionary.

    Parameters:
    - filepath (str): Location of the .geo file to parse.

    Returns:
    - dict: A dictionary containing the general model information and parameters for each rotary axis link.
            Returns an empty dictionary if the file is not found or an error occurs during parsing.
    
    The dictionary includes:
    - RcsVersion (str)
    - ManipulatorName (str)
    - Manufacturer (str)
    - SupportedController (str)
    - Links (dict): Each link name maps to its parameters as a sub-dictionary.
    """
    # Check file existence
    if not os.path.isfile(filepath):
        print(f"File not found: {filepath}")
        return {}
    try:
        # Parse the XML file
        tree = ET.parse(filepath)
        root = tree.getroot()
    except ET.ParseError as e:
        print(f"XML parsing error in file '{filepath}': {e}")
        return {}
    except Exception as e:
        print(f"Unexpected error while reading '{filepath}': {e}")
        return {}

    # Extract general information
    rcs_version = root.findtext(".//RcsVersion", default="N/A")
    manipulator = root.find(".//ArticulatedRobotManipulator")
    manipulator_name = manipulator.findtext("Name", default="N/A") if manipulator is not None else "N/A"
    manufacturer = root.findtext(".//Manufacturer", default="N/A")
    supported_controller = root.findtext(".//SupportedController", default="N/A")

    # Extract link and axis parameter information
    links_info = {}
    for rotary_axis in root.findall(".//Link_RotaryAxis"):
        name = rotary_axis.findtext("Name", default="Unnamed_Link")
        parameters = {}

        all_elements = rotary_axis.find(".//LinkParameters/AllElements")
        if all_elements is not None:
            for param in all_elements.findall("Parameter"):
                param_name = param.findtext("Name", default="Unknown_Parameter")
                param_value_str = param.findtext("Value", default="0.0")
                try:
                    param_value = float(param_value_str)
                except ValueError:
                    print(f"Warning: Invalid float value for parameter '{param_name}' in link '{name}': '{param_value_str}'")
                    param_value = None  # Set to None to indicate parsing error
                parameters[param_name] = param_value

        links_info[name] = parameters

    # Construct and return the final dictionary
    robot_data = {
        "RcsVersion": rcs_version,
        "ManipulatorName": manipulator_name,
        "Manufacturer": manufacturer,
        "SupportedController": supported_controller,
        "Links": links_info
    }

    return robot_data

def export_geomodel_json(robot_data= {}, output_path= 'models/default_robot.json'):
    """
    Exports a dictionary containing robot model data to a JSON file.

    Parameters:
    - robot_data (dict): Dictionary containing the robot model data to be exported.
    - output_path (str): Full or relative path to the output JSON file.

    Returns:
    - bool: True if the export was successful, False otherwise.
    
    Notes:
    - If the robot_data does not include 'ManipulatorName', the function will abort and return False.
    - The function will create directories as needed if the specified path does not exist.
    """
    manipulator_name = robot_data.get("ManipulatorName")
    if not manipulator_name:
        print("Export failed: 'ManipulatorName' not found in robot data.")
        return False

    # If output_path is a directory, generate a file name
    if os.path.isdir(output_path) or output_path.endswith(os.sep):
        output_path = os.path.join(output_path, f"{manipulator_name}_data.json")

    # Ensure the output directory exists
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    try:
        with open(output_path, "w", encoding="utf-8") as json_file:
            json.dump(robot_data, json_file, indent=4)
        print(f"\nData exported successfully to '{output_path}'")
        return True
    except Exception as e:
        print(f"Failed to export data to JSON: {e}")
        return False




if __name__ == '__main__':

    # Extraer la información del archivo .geo
    robot_data = load_geomodel('models/R2000iB_200R.geo')

    # Guardar Json
    export_geomodel_json(robot_data)