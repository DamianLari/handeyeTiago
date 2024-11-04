import csv
import numpy as np
from scipy.spatial.transform import Rotation as R


def adjust_aruco_pose_list(transla,rota):
    adjusted_list = []
    for transla in aruco_transla_list:
        adjusted_transla = (-transla[1], -transla[2], transla[0])
        adjusted_transla_list.append(adjusted_transla)
    for pose in adjusted_rota:
        adjusted_rota = (-transla[1], -transla[2], transla[0])
        adjusted_rota_list.append(adjusted_rota)
    return adjusted_transla_list, adjusted_rota_list
    

def adjust_aruco_pose(transla,rota):
    adjusted_transla = (-transla[1], -transla[2], transla[0])
    adjusted_rota = (-rota[1], -rota[2], rota[0])
    return adjusted_transla, adjusted_rota


def angular_distance(target_rota, rota_list):
    target_rotation = R.from_euler('ZYX', target_rota)
    rota_distances = []
    for rota in rota_list:
        rotation = R.from_euler('ZYX', rota)
        relative_rotation = target_rotation.inv() * rotation
        angle_distance = relative_rotation.magnitude()
        rota_distances.append(angle_distance)
    return np.argmin(rota_distances)


def process_csv(input_file, output_file):
    with open(input_file, mode='r') as infile, open(output_file, mode='w', newline='') as outfile:
        reader = csv.DictReader(infile)
        fieldnames = reader.fieldnames
        writer = csv.DictWriter(outfile, fieldnames=fieldnames)
        writer.writeheader()

        for row in reader:
            tag_pose = (float(row['tag_tx']), float(row['tag_ty']), float(row['tag_tz']))
            tag_rota = (float(row['tag_rx']), float(row['tag_ry']), float(row['tag_rz']))

            new_transla, new_rota = adjust_aruco_pose([float(row['gripper_tx']),float(row['gripper_ty']),float(row['gripper_tz'])], [float(row['gripper_rx']),float(row['gripper_rx']),float(row['gripper_rx'])])
            
            row['gripper_tx'], row['gripper_ty'], row['gripper_tz']  = new_transla
            row['gripper_rx'], row['gripper_ry'], row['gripper_rz'] = new_rota

            writer.writerow(row)

input_file = 'merged_poses.csv' 
output_file = 'merged_poses_new.csv'
#aruco_pose_list = to_the_camera()

process_csv(input_file, output_file)
