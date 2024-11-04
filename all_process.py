import cv2
import os
from TB_tag_detection import TagPoseProvider, CameraConfig, TagConfig, TagDetection
import numpy as np
import argparse
import shutil
import csv

def transformation_matrix(rep_a, rep_b):
    """
    Calcule la matrice de transformation entre deux repères.
    
    Args:
        rep_a: Liste des axes du repère A (ex: ['x', 'y', 'z'])
        rep_b: Liste des axes du repère B avec les transformations (ex: ['-x', '-z', '-y'])
        
    Returns:
        Matrice de transformation 3x3.
    """
    axes = {'x': 0, 'y': 1, 'z': 2}
    transform = {'x': 1, 'y': 1, 'z': 1, '-x': -1, '-y': -1, '-z': -1}

    matrix = np.zeros((3, 3))
    
    for i, axis in enumerate(rep_a):
        sign = transform[rep_b[i]]
        idx = axes[rep_b[i].replace('-', '')]
        matrix[idx, i] = sign

    return matrix

def transform_point(point, matrix, offset):
    """
    Transforme un point selon la matrice de transformation donnée et applique un décalage.
    
    Args:
        point: Coordonnées du point dans le repère source (liste ou array de taille 3).
        matrix: Matrice de transformation 3x3.
        offset: Décalage à appliquer après la transformation.
        
    Returns:
        Coordonnées du point dans le repère cible.
    """
    point = np.array(point).flatten()  # S'assurer que le point est un vecteur de forme (3,)
    transformed_point = np.dot(matrix, point)
    return transformed_point + offset

def transform_rotation(rotation, matrix):
    """
    Transforme une rotation selon la matrice de transformation donnée.
    
    Args:
        rotation: Vecteur de rotation (Rodrigues) dans le repère source.
        matrix: Matrice de transformation 3x3.
        
    Returns:
        Vecteur de rotation transformé dans le repère cible.
    """
    rotation_matrix, _ = cv2.Rodrigues(rotation)
    transformed_rotation_matrix = np.dot(matrix, rotation_matrix)
    transformed_rotation, _ = cv2.Rodrigues(transformed_rotation_matrix)
    return transformed_rotation

# Définition des repères
rep_b = ['x', 'y', 'z']  # Repère de l'AR marker
rep_a = ['-x', '-z', '-y']  # Repère du gripper

# Calcul de la matrice de transformation
transformation = transformation_matrix(rep_a, rep_b)
#print("Matrice de transformation :")
#print(transformation)

parser = argparse.ArgumentParser(description='Estimation de la pose d\'un tag et enregistrement dans un fichier CSV')
parser.add_argument('--image_folder', type=str, default='images', help='le dossier contenant les images')
parser.add_argument('--calibration_file', type=str, default='calib_data.json', help='le fichier de calibration de la caméra (intrinsèque)')
parser.add_argument('--tag_id', type=int, default=None, help='l\'identifiant du tag à détecter')
parser.add_argument('--csv_filename', type=str, default='tag_pose.csv', help='le fichier CSV pour enregistrer les poses')

args = parser.parse_args()
csv_filename = args.csv_filename
image_folder = args.image_folder
calibration_file = args.calibration_file
tag_id = args.tag_id
tag_id = 11
file_exists = os.path.isfile(csv_filename)

tag_detection_provider = TagPoseProvider()

calib_data = tag_detection_provider.load_calib_data(calibration_file)
tag_detection_provider.set_calibration_params(*calib_data)

gripper_position_aruco = np.array([0, -0.25, 0])
image_count = 0

output_image_path = 'newRepere_images'

if os.path.exists(output_image_path):
    shutil.rmtree(output_image_path)
os.makedirs(output_image_path)

if os.path.exists(csv_filename):
    os.remove(csv_filename)

with open(csv_filename, 'a', newline='') as csvfile:
    fieldnames = ['image_file', 'tag_id', 'tx', 'ty', 'tz', 'rx', 'ry', 'rz']
    #fieldnames = ['image_file', 'tag_id', 'tx', 'ty', 'tz', 'gripper_tx', 'gripper_ty', 'gripper_tz', 'rx', 'ry', 'rz', 'gripper_rx', 'gripper_ry', 'gripper_rz']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()

    for filename in os.listdir(image_folder):
        if filename.endswith(".jpg") or filename.endswith(".png"):
            image_path = os.path.join(image_folder, filename)
            image = cv2.imread(image_path)
            if image is None:
                print(f"Erreur : Impossible de lire l'image {image_path}")
                continue

            corners, ids, rvecs, tvecs = tag_detection_provider.detect_marker(image, 'aruco', 0.181, 'DICT_4X4_100')
            
            if ids is not None:
                #ids, rota, tvecs_camera = tag_detection_provider.calculate_positions_in_camera_frame(ids, rvecs, tvecs)
                
                for i in range(len(ids)):
                    if tag_id is None or ids[i] == tag_id:
                        rvec = rvecs[i]
                        tvec = tvecs[i]
                        #print(f"Tag ID {ids[i]} : rvec = {rvec}, tvec = {tvec}")
                        rotation_matrix, _ = cv2.Rodrigues(rvec)
                        translation_vector = tvec.reshape((3, 1))
                        gripper_position_camera = np.dot(rotation_matrix, gripper_position_aruco.reshape((3, 1))) + translation_vector
                        #print(" gripper_position_camera : ", gripper_position_camera.flatten())


                        # Ajouter un cercle sur l'image à l'endroit de gripper_position_camera
                        point_2d, _ = cv2.projectPoints(gripper_position_camera.T, np.zeros((3,1)), np.zeros((3,1)), *calib_data)
                        point_2d = tuple(point_2d[0][0].astype(int))
                        cv2.circle(image, point_2d, 5, (0, 255, 0), -1)

                        gripper_rotation_camera = transform_rotation(rvec, transformation)
                        #print(" gripper_rotation_camera : ", gripper_rotation_camera.flatten())
                        # Affichage du trièdre du gripper
                        axis_length = 0.1  # longueur des axes du trièdre
                        axes = np.float32([[axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]]).reshape(-1, 3)
                        imgpts, _ = cv2.projectPoints(axes, gripper_rotation_camera, gripper_position_camera, calib_data[0], calib_data[1])

                        # Dessiner les axes sur l'image
                        imgpts = imgpts.astype(int)
                        origin = point_2d
                        image = cv2.line(image, origin, tuple(imgpts[0].ravel()), (0, 0, 255), 5)  # Axe X en rouge
                        image = cv2.line(image, origin, tuple(imgpts[1].ravel()), (0, 255, 0), 5)  # Axe Y en vert
                        image = cv2.line(image, origin, tuple(imgpts[2].ravel()), (255, 0, 0), 5)  # Axe Z en bleu

                        # Enregistrer l'image modifiée
                        output_image_file = os.path.join(output_image_path, filename)
                        cv2.imwrite(output_image_file, image)

                        # Enregistrer les données dans le fichier CSV
                        writer.writerow({
                            'image_file': filename,
                            'tag_id': ids[i],
                            #'tx': tvec[0][0], 'ty': tvec[0][1], 'tz': tvec[0][2],
                            'tx': gripper_position_camera[0][0], 'ty': gripper_position_camera[1][0], 'tz': gripper_position_camera[2][0],
                            #'rx': rvec[0][0], 'ry': rvec[0][1], 'rz': rvec[0][2],
                            'rx': gripper_rotation_camera[0][0], 'ry': gripper_rotation_camera[1][0], 'rz': gripper_rotation_camera[2][0]
                        })

                        image_count += 1

if tag_id is not None:
    print(f"Le tag {tag_id} a été détecté {image_count} fois.")
else:
    print(f"Les poses de tous les tags ont été enregistrées à partir de {image_count} images.")

import csv
import os
import argparse

class MergePose:
    def __init__(self, filename1, filename2):
        self.filename1 = filename1
        self.filename2 = filename2
        
    def isFormatGood(self, filename, required_fields):
        with open(filename, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            return all(field in reader.fieldnames for field in required_fields)
    
    def read_merged_poses(self,merged_filename):
        if not merged_filename:
            raise ValueError("Le nom du fichier fusionné n'est pas défini")
        
        required_fields = [
            'image_file', 'tag_id', 'tag_tx', 'tag_ty', 'tag_tz', 
            'tag_rx', 'tag_ry', 'tag_rz', 'gripper_tx', 'gripper_ty', 
            'gripper_tz', 'gripper_rx', 'gripper_ry', 'gripper_rz'
        ]
        
        if not self.isFormatGood(merged_filename, required_fields):
            raise ValueError("Le format du fichier CSV fusionné n'est pas correct")
        
        merged_poses = {}
        with open(merged_filename, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                merged_poses[row['image_file']] = {
                    'tag_id': int(row['tag_id']),
                    'tag_pose': {
                        'tx': float(row['tag_tx']),
                        'ty': float(row['tag_ty']),
                        'tz': float(row['tag_tz']),
                        'rx': float(row['tag_rx']),
                        'ry': float(row['tag_ry']),
                        'rz': float(row['tag_rz'])
                    },
                    'gripper_pose': {
                        'tx': float(row['gripper_tx']),
                        'ty': float(row['gripper_ty']),
                        'tz': float(row['gripper_tz']),
                        'rx': float(row['gripper_rx']),
                        'ry': float(row['gripper_ry']),
                        'rz': float(row['gripper_rz'])
                    }
                }
        return merged_poses
    
    
    def merge_poses(self):
        required_fields1 = ['tx', 'ty', 'tz', 'rx', 'ry', 'rz']
        required_fields2 = ['image_file', 'tag_id', 'tx', 'ty', 'tz', 'rx', 'ry', 'rz']
        """
        if not (self.isFormatGood(self.filename1, required_fields1) and self.isFormatGood(self.filename2, required_fields2)):
            raise ValueError("Le format des fichiers CSV n'est pas correct")
        """
        poses1 = self.read_pose_csv(self.filename1)
        poses2 = self.read_image_csv(self.filename2)

        merged_poses = self.associate_poses(poses1, poses2)
        return merged_poses

    def read_pose_csv(self, filename):
        poses = []
        with open(filename, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            if not all(field in reader.fieldnames for field in ['tx', 'ty', 'tz', 'rx', 'ry', 'rz']):
                raise ValueError(f"Fichier {filename} n'a pas les champs nécessaires : 'tx', 'ty', 'tz', 'rx', 'ry', 'rz'")
            for row in reader:
                pose = {
                    'tx': float(row['tx']),
                    'ty': float(row['ty']),
                    'tz': float(row['tz']),
                    'rx': float(row['rx']),
                    'ry': float(row['ry']),
                    'rz': float(row['rz'])
                }
                poses.append(pose)
        return poses

    def read_image_csv(self, filename):
        images = []
        with open(filename, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            if not all(field in reader.fieldnames for field in ['image_file', 'tag_id', 'tx', 'ty', 'tz', 'rx', 'ry', 'rz']):
                raise ValueError(f"Fichier {filename} n'a pas les champs nécessaires : 'image_file', 'tag_id', 'tx', 'ty', 'tz', 'rx', 'ry', 'rz'")
            for row in reader:
                image_data = {
                    'image_file': row['image_file'].strip(), 
                    'tag_id': int(row['tag_id']),
                    'tx': float(row['tx']),
                    'ty': float(row['ty']),
                    'tz': float(row['tz']),
                    'rx': float(row['rx']),
                    'ry': float(row['ry']),
                    'rz': float(row['rz'])
                }
                images.append(image_data)
        return images

    def associate_poses(self, poses, images):
        associated_poses = {}
        for image in images:
            image_basename = os.path.splitext(os.path.basename(image['image_file']))[0]
            image_number_str = ''.join(filter(str.isdigit, image_basename))
            try:
                image_number = int(image_number_str)
            except ValueError:
                print(f"Impossible de convertir {image_basename} en nombre")
                continue
            
            if image_number - 1 < len(poses):  
                gripper_pose = poses[image_number - 1] 
                image_file_with_extension = f"{image_basename}.jpg"  
                associated_poses[image_file_with_extension] = {
                    'tag_id': image['tag_id'],
                    'gripper_pose': gripper_pose,
                    'tag_pose': {
                        'tx': image['tx'],
                        'ty': image['ty'],
                        'tz': image['tz'],
                        'rx': image['rx'],
                        'ry': image['ry'],
                        'rz': image['rz']
                    }
                }
            else:
                print(f"Aucune pose de gripper correspondante pour l'image {image['image_file']}")
        return associated_poses
    
    def save_to_csv(self, output_filename):
        if os.path.exists(output_filename):
            os.remove(output_filename)
        merged_poses = self.merge_poses()
        with open(output_filename, 'w', newline='') as csvfile:
            fieldnames = ['image_file', 'tag_id', 'tag_tx', 'tag_ty', 'tag_tz', 'tag_rx', 'tag_ry', 'tag_rz',
                          'gripper_tx', 'gripper_ty', 'gripper_tz', 'gripper_rx', 'gripper_ry', 'gripper_rz']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for image_file, data in merged_poses.items():
                row = {
                    'image_file': image_file,
                    'tag_id': data['tag_id'],
                    'tag_tx': data['tag_pose']['tx'],
                    'tag_ty': data['tag_pose']['ty'],
                    'tag_tz': data['tag_pose']['tz'],
                    'tag_rx': data['tag_pose']['rx'],
                    'tag_ry': data['tag_pose']['ry'],
                    'tag_rz': data['tag_pose']['rz'],
                    'gripper_tx': data['gripper_pose']['tx'],
                    'gripper_ty': data['gripper_pose']['ty'],
                    'gripper_tz': data['gripper_pose']['tz'],
                    'gripper_rx': data['gripper_pose']['rx'],
                    'gripper_ry': data['gripper_pose']['ry'],
                    'gripper_rz': data['gripper_pose']['rz']
                }
                writer.writerow(row)
                
    def get_certain_tag_pose(self, image_file):
        merged_poses = self.merge_poses()
        #print("Clés disponibles dans merged_poses:", list(merged_poses.keys())) 
        image_file = image_file.strip()  
        if image_file not in merged_poses:
            raise KeyError(f"Le fichier image '{image_file}' n'existe pas dans merged_poses")
        return merged_poses[image_file]['tag_pose']
    
    def get_certain_gripper_pose(self, image_file):
        merged_poses = self.merge_poses()
        #print("Clés disponibles dans merged_poses:", list(merged_poses.keys()))  
        image_file = image_file.strip()  
        if image_file not in merged_poses:
            raise KeyError(f"Le fichier image '{image_file}' n'existe pas dans merged_poses")
        return merged_poses[image_file]['gripper_pose']


def main():
    parser = argparse.ArgumentParser(description='Estimation de la pose d\'un tag et enregistrement dans un fichier CSV')
    parser.add_argument('--gripper_pose_csv', type=str, help='Fichier CSV contenant les poses du gripper')
    parser.add_argument('--tag_pose_csv', type=str, help='Fichier CSV contenant les poses du tag')
    parser.add_argument('--output_csv', type=str, help='Fichier CSV de sortie')

    args = parser.parse_args()

    merger = MergePose( args.gripper_pose_csv, args.tag_pose_csv)
    merger.save_to_csv(args.output_csv)
    #print(merger.get_certain_tag_pose('image_0001.jpg'))

if __name__ == "__main__":
    main()
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2
import argparse
import csv
from TsaiCalib import TsaiHandEyeCalibration
import json
import os
import shutil

class EyeInHandCalib:
    def __init__(self, csv_file_path, output_dir, arm):
        self.csv_file_path = csv_file_path
        self.merged_poses = self.read_merged_poses()
        self.output_dir = output_dir
        self.arm = arm
    def read_merged_poses(self):
        merged_poses = {}
        with open(self.csv_file_path, mode='r') as csvfile:
            csv_reader = csv.DictReader(csvfile)
            for row in csv_reader:
                image_file = row['image_file']
                merged_poses[image_file] = {
                    'tag_pose': {
                        'tx': float(row['tag_tx']),
                        'ty': float(row['tag_ty']),
                        'tz': float(row['tag_tz']),
                        'rx': float(row['tag_rx']),
                        'ry': float(row['tag_ry']),
                        'rz': float(row['tag_rz'])
                    },
                    'gripper_pose': {
                        'tx': float(row['gripper_tx']),
                        'ty': float(row['gripper_ty']),
                        'tz': float(row['gripper_tz']),
                        'rx': float(row['gripper_rx']),
                        'ry': float(row['gripper_ry']),
                        'rz': float(row['gripper_rz'])
                    }
                }
        return merged_poses
        
    def extract_rot_matrix(self, pose):
        return R.from_euler('xyz', [pose['rx'], pose['ry'], pose['rz']], degrees=False).as_matrix()
                
    def extract_translation(self, pose):
        return np.array([pose['tx'], pose['ty'], pose['tz']])

    def get_eye_to_hand(self):
        all_R_cam_to_target = []
        all_t_cam_to_target = []
        all_R_base_to_gripper = []
        all_t_base_to_gripper = []
        
        for image_file in self.merged_poses:
            tag_pose_from_camera = self.merged_poses[image_file]['tag_pose']
            gripper_pose_from_base = self.merged_poses[image_file]['gripper_pose']
            
            R_base_to_gripper = self.extract_rot_matrix(gripper_pose_from_base)
            t_base_to_gripper = self.extract_translation(gripper_pose_from_base).reshape(3, 1)
           
            R_cam_to_target = self.extract_rot_matrix(tag_pose_from_camera)
            t_cam_to_target = self.extract_translation(tag_pose_from_camera).reshape(3, 1)
            
            all_R_cam_to_target.append(R_cam_to_target)
            all_t_cam_to_target.append(t_cam_to_target)
            
            all_R_base_to_gripper.append(R_base_to_gripper)
            all_t_base_to_gripper.append(t_base_to_gripper)

        all_R_cam_to_target = np.array(all_R_cam_to_target)
        all_t_cam_to_target = np.array(all_t_cam_to_target)
        all_R_base_to_gripper = np.array(all_R_base_to_gripper)
        all_t_base_to_gripper = np.array(all_t_base_to_gripper)

        # Tsai calibration
        self.R_cam_to_base, self.t_cam_to_base = TsaiHandEyeCalibration(
        all_R_base_to_gripper, all_t_base_to_gripper, 
        all_R_cam_to_target, all_t_cam_to_target
        )
       
        self. R_cam_to_base = R.from_matrix(self.R_cam_to_base).as_euler('xyz', degrees=False)
        print("Rotation matrix from camera to base:", self.R_cam_to_base)
        print("Translation vector from camera to base:", self.t_cam_to_base)
        
        file_exists = os.path.isfile(self.output_dir)
        with open(self.output_dir, mode='a', newline='') as csvfile:
            fieldnames = ['armAndTag', 'cRb_x', 'cRb_y', 'cRb_z', 'cTb_x', 'cTb_y', 'cTb_z']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()  # Écrire l'en-tête seulement si le fichier n'existe pas
            armAndTag = self.arm + f"{len(all_R_base_to_gripper)}"
            writer.writerow({
                'armAndTag': armAndTag,
                'cRb_x': self.R_cam_to_base[0], 'cRb_y': self.R_cam_to_base[1], 'cRb_z': self.R_cam_to_base[2],
                'cTb_x': self.t_cam_to_base[0], 'cTb_y': self.t_cam_to_base[1], 'cTb_z': self.t_cam_to_base[2]
            })
       
    def get_transformation_matrix(self, rota, transla):
        transformation_matrix = np.identity(4)
        transformation_matrix[:3, :3] = rota

        if isinstance(transla, list):
            transla = np.array(transla).reshape(3, 1)
        if transla.shape == (3, 3):
            translation_3x1 = transla[:, 0]
        elif transla.shape == (3, 1):
            translation_3x1 = transla
        else:
            raise ValueError("La matrice de translation doit être de forme 3x3, 3x1 ou une liste [x, y, z]")

        transformation_matrix[:3, 3] = translation_3x1.flatten()
        return transformation_matrix

"""
parser = argparse.ArgumentParser(description='Estimation de la pose d\'un tag et enregistrement dans un fichier CSV')
parser.add_argument('--csv_file', type=str, help='Chemin du fichier CSV contenant les poses fusionnées')
args = parser.parse_args()
"""
output_image_path = 'calib_results.csv'
"""
if os.path.exists(output_image_path):
    os.remove(output_image_path)
    """
#eye_in_hand_calib = EyeInHandCalib(args.csv_file)
eye_in_hand_calib = EyeInHandCalib("merged_poses.csv",output_image_path, "Left")
eye_in_hand_calib.get_eye_to_hand()

