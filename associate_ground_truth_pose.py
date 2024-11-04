import csv
import numpy as np
from scipy.spatial.transform import Rotation as R

aruco_pose_list = [
                [0, 0, 0],    
                [0.5, 0, 0], 
                [1, 0, 0],
                [-0.5, 0, 0],
                [-1, 0, 0],

                [0, 0.5, 0],  
                [0, 0.75, 0],
                [0, -0.5, 0],
                [0, -0.75, 0],

                [0, 0, 0.5], 
                [0, 0, 1],
                [0, 0, -0.5],
                [0, 0, -1]
            ]

aruco_rota_list = [
            [0, 0, 0],          # No rotation
            [np.pi/6, 0, 0],    # Roll 30°
            [np.pi/4, 0, 0],    # Roll 45°
            [np.pi/2, 0, 0],    # Roll 90°
            [np.pi, 0, 0],      # Roll 180°
            [-np.pi/2, 0, 0],   # Roll -90°
            [0, np.pi/6, 0],    # Pitch 30°
            [0, np.pi/4, 0],    # Pitch 45°
            [0, np.pi/2, 0],    # Pitch 90°
            [0, np.pi, 0],      # Pitch 180°
            [0, -np.pi/2, 0],   # Pitch -90°
            [0, 0, np.pi/6],    # Yaw 30°
            [0, 0, np.pi/4],    # Yaw 45°
            [0, 0, np.pi/2],    # Yaw 90°
            [0, 0, np.pi],      # Yaw 180°
            [0, 0, -np.pi/2],   # Yaw -90°
        ]
aruco_refering_pose=[2,0,0,0,0,0]

def to_the_camera():
    tag_pose = []
    for pose in aruco_pose_list:
        tag_pose.append((pose[0]+2,pose[1],pose[2]))
    return tag_pose

def to_the_camera():
    tag_pose = []
    tag_rota = []
    for pose in aruco_pose_list:
        tag_pose.append((pose[0]+2,pose[1],pose[2]))
    for rota in aruco_rota_list:
        tag_rota.append((rota[0]+3.14159 ,rota[1]+ 1.5708,rota[2]))
        #tag_rota.append((rota[0] ,rota[1],rota[2]))
    return tag_pose,tag_rota

def adjust_aruco_pose_list(aruco_pose_list):
    adjusted_list = []
    for pose in aruco_pose_list:
        adjusted_pose = (-pose[1], -pose[2], pose[0])
        adjusted_list.append(adjusted_pose)
    return adjusted_list

def adjust_aruco_rota_list(aruco_rota_list):
    adjusted_rota_list = []
    for rota in aruco_rota_list:
        adjusted_rota = (-rota[1], -rota[2], rota[0])  # supposé être la même chose que pour les translations
        adjusted_rota_list.append(adjusted_rota)
    return adjusted_rota_list

"""
def find_closest_pose(tag_pose, aruco_pose_list):
    tag_x, tag_y, tag_z = tag_pose
    distances = [np.linalg.norm(np.array([tag_x, tag_y, tag_z]) - np.array(aruco_pose)) for aruco_pose in aruco_pose_list]
    min_index = np.argmin(distances)
    return aruco_pose_list[min_index]
"""

def angular_distance(target_rota, rota_list):
    target_rotation = R.from_euler('ZYX', target_rota)
    rota_distances = []
    for rota in rota_list:
        rotation = R.from_euler('ZYX', rota)
        relative_rotation = target_rotation.inv() * rotation
        angle_distance = relative_rotation.magnitude()
        rota_distances.append(angle_distance)
    return np.argmin(rota_distances)

"""  
def find_closest_pose(tag_pose, aruco_pose_list):
    tag_x, tag_y, tag_z = tag_pose
    distances = [np.linalg.norm(np.array([tag_x, tag_y, tag_z]) - np.array(aruco_pose)) for aruco_pose in aruco_pose_list]
    min_index = np.argmin(distances)
    return aruco_pose_list[min_index], aruco_rota_list[min_index]
"""

def find_closest_pose(tag_pose, tag_rota, aruco_pose_list, aruco_rota_list):
    tag_x, tag_y, tag_z = tag_pose
    distances = [np.linalg.norm(np.array([tag_x, tag_y, tag_z]) - np.array(aruco_pose)) for aruco_pose in aruco_pose_list]
    min_index_translation = np.argmin(distances)
    min_index_rotation = angular_distance(tag_rota, aruco_rota_list)
    
    return aruco_pose_list[min_index_translation], aruco_rota_list[min_index_rotation]


def find_best_component(tag_value, component_index, aruco_pose_list):
   
    component_values = [pose[component_index] for pose in aruco_pose_list]
    distances = [abs(tag_value - value) for value in component_values]
    min_index = np.argmin(distances)
    return component_values[min_index]

def find_closest_pose_per_component(tag_pose, aruco_pose_list):
    tag_x, tag_y, tag_z = tag_pose

    best_x = find_best_component(tag_x, 0, aruco_pose_list)
    best_y = find_best_component(tag_y, 1, aruco_pose_list)
    best_z = find_best_component(tag_z, 2, aruco_pose_list)

    return best_x, best_y, best_z

def process_csv(input_file, output_file):
    with open(input_file, mode='r') as infile, open(output_file, mode='w', newline='') as outfile:
        reader = csv.DictReader(infile)
        fieldnames = reader.fieldnames
        writer = csv.DictWriter(outfile, fieldnames=fieldnames)
        writer.writeheader()

        for row in reader:
            tag_pose = (float(row['tag_tx']), float(row['tag_ty']), float(row['tag_tz']))
            tag_rota = (float(row['tag_rx']), float(row['tag_ry']), float(row['tag_rz']))

            #closest_pose = find_closest_pose(tag_pose, aruco_pose_list)
            #closest_transla, closest_rota = find_closest_pose(tag_pose, aruco_pose_list)
            closest_transla, closest_rota = find_closest_pose(tag_pose, tag_rota, aruco_pose_list, aruco_rota_list)
            
            row['gripper_tx'], row['gripper_ty'], row['gripper_tz']  = closest_transla
            row['gripper_rx'], row['gripper_ry'], row['gripper_rz'] = closest_rota

            writer.writerow(row)

input_file = 'merged_poses.csv' 
output_file = 'merged_poses_new.csv'
#aruco_pose_list = to_the_camera()
aruco_pose_list , aruco_rota_list = to_the_camera()
aruco_pose_list = adjust_aruco_pose_list(aruco_pose_list)
aruco_rota_list = adjust_aruco_rota_list(aruco_rota_list)

process_csv(input_file, output_file)
