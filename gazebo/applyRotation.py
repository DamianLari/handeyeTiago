import xml.etree.ElementTree as ET
import numpy as np
import math
# Fonction de rotation en 3D
def rotation_matrix(rx, ry, rz):
    # Matrice de rotation sur l'axe x
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ])
    
    # Matrice de rotation sur l'axe y
    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])
    
    # Matrice de rotation sur l'axe z
    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ])
    
    # Rotation totale en appliquant les rotations dans l'ordre z, y, x
    return Rz @ Ry @ Rx

# Appliquer la rotation sur chaque lien
def apply_rotation_to_sdf(file_path, output_path, rx, ry, rz):
    tree = ET.parse(file_path)
    root = tree.getroot()

    # Créer une matrice de rotation
    rotation_mat = rotation_matrix(rx, ry, rz)
    
    for link in root.findall(".//link"):
        pose = link.find('pose')
        if pose is not None:
            pose_values = list(map(float, pose.text.strip().split()))
            
            # Si le pose contient position (x, y, z) et rotation (roll, pitch, yaw)
            if len(pose_values) == 6:
                x, y, z, roll, pitch, yaw = pose_values
                
                # Position initiale du lien
                position = np.array([x, y, z])
                # Rotation initiale en matrice (de roll, pitch, yaw vers matrice de rotation)
                initial_rotation = rotation_matrix(roll, pitch, yaw)
                
                # Nouvelle position après la rotation
                new_position = rotation_mat @ position
                # Nouvelle rotation totale
                new_rotation = rotation_mat @ initial_rotation
                
                # Extraire les nouveaux angles de rotation roll, pitch, yaw
                new_roll = np.arctan2(new_rotation[2, 1], new_rotation[2, 2])
                new_pitch = np.arctan2(-new_rotation[2, 0], np.sqrt(new_rotation[2, 1]**2 + new_rotation[2, 2]**2))
                new_yaw = np.arctan2(new_rotation[1, 0], new_rotation[0, 0])
                
                # Mettre à jour la pose avec les nouvelles valeurs
                pose.text = f"{new_position[0]} {new_position[1]} {new_position[2]} {new_roll} {new_pitch} {new_yaw}"

    # Enregistrer le fichier SDF modifié
    tree.write(output_path)
    print(f"Le fichier SDF modifié a été enregistré sous '{output_path}'")

# Exécution de la fonction avec les angles de rotation (ex. en radians)
apply_rotation_to_sdf('aruco11.sdf', 'aruco.sdf', rx=0, ry=math.pi, rz=-math.pi/2)
