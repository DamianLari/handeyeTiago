import cv2
import os
from TB_tag_detection import TagPoseProvider, CameraConfig, TagConfig, TagDetection
import numpy as np
import argparse
import shutil
import csv
from scipy.spatial.transform import Rotation as R


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
    point = np.array(point).flatten() 
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

gripper_position_aruco = np.array([0, 0, 0])
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

            undistord_image = tag_detection_provider.correct_image(image)
            #undistord_image = image
            corners, ids, rvecs, tvecs = tag_detection_provider.detect_marker(undistord_image, 'aruco', 0.18, 'DICT_4X4_100')
            if ids is not None:
                #ids, rota, tvecs_camera = tag_detection_provider.calculate_positions_in_camera_frame(ids, rvecs, tvecs)
                
                for i in range(len(ids)):
                    if tag_id is None or ids[i] == tag_id:
                        rvec = rvecs[i]
                        tvec = tvecs[i]

                        corner_pts = corners[i].reshape(-1, 2)  
                        for j, corner in enumerate(corner_pts):
                            corner = tuple(corner.astype(int))
                            cv2.circle(undistord_image, corner, 1, (0, 0, 255), -1) 
                            #undistord_image[corner[1], corner[0]] = (0, 0, 255)
                            #cv2.putText(undistord_image, f"C{j+1}", corner, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                        rotation_matrix, _ = cv2.Rodrigues(rvec)
                        translation_vector = tvec.reshape((3, 1))
                        gripper_position_camera = np.dot(rotation_matrix, gripper_position_aruco.reshape((3, 1))) + translation_vector
                        
                        point_2d, _ = cv2.projectPoints(gripper_position_camera.T, np.zeros((3,1)), np.zeros((3,1)), *calib_data)
                        point_2d = tuple(point_2d[0][0].astype(int))
                        cv2.circle(undistord_image, point_2d, 5, (0, 255, 0), -1)

                        gripper_rotation_camera = transform_rotation(rvec, transformation)
                        
                        axis_length = 0.18
                        """
                        axes = np.float32([[axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]]).reshape(-1, 3)
                        imgpts, _ = cv2.projectPoints(axes, gripper_rotation_camera, gripper_position_camera, calib_data[0], calib_data[1])

                        imgpts = imgpts.astype(int)
                        origin = point_2d
                        image = cv2.line(undistord_image, origin, tuple(imgpts[0].ravel()), (0, 0, 255), 5)  # Axe X en rouge
                        image = cv2.line(undistord_image, origin, tuple(imgpts[1].ravel()), (0, 255, 0), 5)  # Axe Y en vert
                        image = cv2.line(undistord_image, origin, tuple(imgpts[2].ravel()), (255, 0, 0), 5)  # Axe Z en bleu
                        """
                        axis_length = 0.18  # Longueur des axes
                        cv2.drawFrameAxes(undistord_image, calib_data[0], calib_data[1], rvec, tvec, axis_length) 

                        output_image_file = os.path.join(output_image_path, filename)
                        cv2.imwrite(output_image_file, undistord_image)
                        rvec
                        
                        rpy=np.flip(R.from_rotvec(rvec).as_euler("ZYX",degrees=False) ) 

                        writer.writerow({
                            'image_file': filename,
                            'tag_id': ids[i],
                            'tx': tvec[0][0], 'ty': tvec[0][1], 'tz': tvec[0][2],
                            #'tx': gripper_position_camera[0][0], 'ty': gripper_position_camera[1][0], 'tz': gripper_position_camera[2][0],
                            'rx': rpy[0][0], 'ry': rpy[0][1], 'rz': rpy[0][2],
                            #'rx': gripper_rotation_camera[0][0], 'ry': gripper_rotation_camera[1][0], 'rz': gripper_rotation_camera[2][0]
                        })

                        image_count += 1

if tag_id is not None:
    print(f"Le tag {tag_id} a été détecté {image_count} fois.")
else:
    print(f"Les poses de tous les tags ont été enregistrées à partir de {image_count} images.")
