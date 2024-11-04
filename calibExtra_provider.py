import numpy as np
from scipy.spatial.transform import Rotation as R
import csv
import os

def read_merged_poses(csv_file_path):
    merged_poses = []
    with open(csv_file_path, mode='r') as csvfile:
        csv_reader = csv.DictReader(csvfile)
        for row in csv_reader:
            pose_data = {
                'tag_pose': extract_pose_from_row(row, 'tag'),
                'gripper_pose': extract_pose_from_row(row, 'gripper')
            }
            merged_poses.append(pose_data)
    return merged_poses

def extract_pose_from_row(row, prefix):
    return {
        'tx': float(row[f'{prefix}_tx']),
        'ty': float(row[f'{prefix}_ty']),
        'tz': float(row[f'{prefix}_tz']),
        'rx': float(row[f'{prefix}_rx']),
        'ry': float(row[f'{prefix}_ry']),
        'rz': float(row[f'{prefix}_rz'])
    }

def extract_rot_matrix(pose):
    return R.from_euler('ZYX', [pose['rz'], pose['ry'], pose['rx']], degrees=False).as_matrix()

def extract_translation(pose):
    return np.array([pose['tx'], pose['ty'], pose['tz']])

def get_rot_trans( pose):
        R = extract_rot_matrix(pose)
        t = extract_translation(pose).reshape(3, 1)
        return R, t

def getTransformasTxTyTzRPYdeg(H):
    Rot = H[0:3,0:3]
    
    rpy=np.flip(R.from_matrix(Rot).as_euler('ZYX', degrees=True))
    return [H[0,3],H[1,3],H[2,3],rpy[0],rpy[1],rpy[2]]

def getTransformasTxTyTzRPYdeg2(rot,T):
    T=T.reshape(-1)
    rpy=np.flip(R.from_matrix(rot).as_euler('ZYX', degrees=True))
    return np.array([T[0],T[1],T[2],rpy[0],rpy[1],rpy[2]])
def calculate_3d_reprojection_and_angular_error(translation_A_in_C, rotation_A_to_C, 
                                                translation_A_in_B, rotation_A_to_B,
                                                transformation_matrix_C_to_B):
    isPrint = False
    # convertir A dans C en coordonnées homogènes
    point_A_in_C_homogeneous = np.append(translation_A_in_C, 1)

    # appliquer transformation repère C dans B
    point_A_in_B_transformed = transformation_matrix_C_to_B @ point_A_in_C_homogeneous

    # coordonnées cartésiennes 
    point_A_in_B_transformed_cartesian = point_A_in_B_transformed[:3] / point_A_in_B_transformed[3]

    # erreur de reprojection
    reprojection_error = np.linalg.norm(translation_A_in_B - point_A_in_B_transformed_cartesian)

    # Convertir les rotations en quaternions
    rotation_C_to_B = R.from_matrix(transformation_matrix_C_to_B[:3, :3])  # rotation C à B en quaternion
    rotation_A_in_C_quat = R.from_euler('ZYX', np.flip(rotation_A_to_C), degrees=False)  # rotation A dans C en quaternion
    rotation_A_in_B_direct_quat = R.from_rotvec(rotation_A_to_B)  # rotation A dans B en quaternion
    
    # appliquer rotation de C à B à larotation de A dans C
    rotation_A_in_C_transformed_quat = rotation_C_to_B * rotation_A_in_C_quat

    # calculer erreur angulaire
    angular_error = rotation_A_in_C_transformed_quat.inv() * rotation_A_in_B_direct_quat
    angular_error_euler = np.flip(angular_error.as_euler('ZYX', degrees=False))  # Erreur en angles d'Euler
    angular_error_degrees = angular_error.magnitude() * (180 / np.pi)  # Erreur en degrés

    if isPrint:
        print_calculate_3d_reprojection_and_angular_error(point_A_in_C_homogeneous, point_A_in_B_transformed, point_A_in_B_transformed_cartesian, reprojection_error, angular_error_euler, angular_error_degrees)
    if reprojection_error > 10:
        print_calculate_3d_reprojection_and_angular_error(point_A_in_C_homogeneous, point_A_in_B_transformed, point_A_in_B_transformed_cartesian, reprojection_error, angular_error_euler, angular_error_degrees)
        raise (1)
    return reprojection_error, angular_error_euler


def write_to_csv(output_dir, i, arm, R_cam_to_base, t_cam_to_base, mean_3D_error, stdev_3D_error):
    file_exists = os.path.isfile(output_dir)
    with open(output_dir, mode='a', newline='') as csvfile:
        fieldnames = ['armAndTag', 'cRb_x', 'cRb_y', 'cRb_z', 'cTb_x', 'cTb_y', 'cTb_z', 'mean_3D_error', 'stdev_3D_error']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        if not file_exists:
            writer.writeheader()
        armAndTag = arm + f"{i}"
        writer.writerow({
            'armAndTag': armAndTag,
            'cRb_x': R_cam_to_base[0], 'cRb_y': R_cam_to_base[1], 'cRb_z': R_cam_to_base[2],
            'cTb_x': t_cam_to_base[0], 'cTb_y': t_cam_to_base[1], 'cTb_z': t_cam_to_base[2],
            'mean_3D_error': mean_3D_error, 'stdev_3D_error': stdev_3D_error
        })

def remove_outliers(data, m=2):
        mean = np.mean([d['mean_error'] for d in data])
        stdev = np.std([d['mean_error'] for d in data])
        filtered_data = [d for d in data if abs(d['mean_error'] - mean) < m * stdev]
        return filtered_data

def calculate_barycenter(merged_poses, indices):
    tx = np.mean([merged_poses[i]['gripper_pose']['tx'] for i in indices])
    ty = np.mean([merged_poses[i]['gripper_pose']['ty'] for i in indices])
    tz = np.mean([merged_poses[i]['gripper_pose']['tz'] for i in indices])
    rx = np.mean([merged_poses[i]['gripper_pose']['rx'] for i in indices])
    ry = np.mean([merged_poses[i]['gripper_pose']['ry'] for i in indices])
    rz = np.mean([merged_poses[i]['gripper_pose']['rz'] for i in indices])

    gripper_barycenter = {
        'tx': tx, 'ty': ty, 'tz': tz,
        'rx': rx, 'ry': ry, 'rz': rz
    }

    tx = np.mean([merged_poses[i]['tag_pose']['tx'] for i in indices])
    ty = np.mean([merged_poses[i]['tag_pose']['ty'] for i in indices])
    tz = np.mean([merged_poses[i]['tag_pose']['tz'] for i in indices])
    rx = np.mean([merged_poses[i]['tag_pose']['rx'] for i in indices])
    ry = np.mean([merged_poses[i]['tag_pose']['ry'] for i in indices])
    rz = np.mean([merged_poses[i]['tag_pose']['rz'] for i in indices])

    tag_barycenter = {
        'tx': tx, 'ty': ty, 'tz': tz,
        'rx': rx, 'ry': ry, 'rz': rz
    }

    return gripper_barycenter, tag_barycenter

#======================Partie Affichage======================*
def print_get_eye_to_hand(indices,R_cam_to_base,t_cam_to_base,mean_3D_error,stdev_3D_error):
    print(f"============ Calibration results with {len(indices)} images =======")
    print(f"Rotation matrix from camera to base: {R_cam_to_base}")
    print(f"Translation vector from camera to base: {t_cam_to_base}")
    print(f"Mean 3D error: {mean_3D_error}")
    print(f"Standard deviation of 3D error: {stdev_3D_error}")

def print_calculate_3d_reprojection_and_angular_error(point_A_in_C_homogeneous,point_A_in_B_transformed,point_A_in_B_transformed_cartesian,reprojection_error,angular_error_euler,angular_error_degrees):
    print("point_A_in_C_homogeneous",point_A_in_C_homogeneous)
    print("point_A_in_B_transformed",point_A_in_B_transformed)
    print("point_A_in_B_transformed_cartesian",point_A_in_B_transformed_cartesian)
    print("reprojection_error",reprojection_error)
    print("angular_error_euler",angular_error_euler)
    print("angular_error_degrees",angular_error_degrees)