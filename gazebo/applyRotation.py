import xml.etree.ElementTree as ET
import numpy as np
import math

def rotation_matrix(rx, ry, rz):
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ])
    
    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])
    
    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ])
    
    return Rz @ Ry @ Rx

def apply_rotation_to_sdf(file_path, output_path, rx, ry, rz):
    tree = ET.parse(file_path)
    root = tree.getroot()

    rotation_mat = rotation_matrix(rx, ry, rz)
    
    for link in root.findall(".//link"):
        pose = link.find('pose')
        if pose is not None:
            pose_values = list(map(float, pose.text.strip().split()))
            
            if len(pose_values) == 6:
                x, y, z, roll, pitch, yaw = pose_values
                
                position = np.array([x, y, z])
                initial_rotation = rotation_matrix(roll, pitch, yaw)
                
                new_position = rotation_mat @ position
                new_rotation = rotation_mat @ initial_rotation
                
                new_roll = np.arctan2(new_rotation[2, 1], new_rotation[2, 2])
                new_pitch = np.arctan2(-new_rotation[2, 0], np.sqrt(new_rotation[2, 1]**2 + new_rotation[2, 2]**2))
                new_yaw = np.arctan2(new_rotation[1, 0], new_rotation[0, 0])
                
                pose.text = f"{new_position[0]} {new_position[1]} {new_position[2]} {new_roll} {new_pitch} {new_yaw}"

    tree.write(output_path)
    print(f"Le fichier SDF modifié a été enregistré sous '{output_path}'")

# Exécution de la fonction avec les angles de rotation (ex. en radians)
apply_rotation_to_sdf('aruco11.sdf', 'aruco.sdf', rx=0, ry=math.pi, rz=-math.pi/2)

