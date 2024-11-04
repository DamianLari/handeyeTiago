from scipy.spatial.transform import Rotation as R
import numpy as np
import csv
import os
from TsaiCalib import TsaiHandEyeCalibration
import random
import copy
from sklearn.linear_model import RANSACRegressor
from calibExtra_provider import *
import math3d as m3d

class EyeInHandCalib:
    def __init__(self, csv_file_path, output_dir, arm):
        self.csv_file_path = csv_file_path  
        self.merged_poses = read_merged_poses(self.csv_file_path)
        self.output_dir = output_dir
        self.arm = arm
        self.best_combinations = []
    
     
    def calculate_reprojection_error(self, all_R_base_to_gripper, all_t_base_to_gripper, all_R_cam_to_target, all_t_cam_to_target):
        errors = []
        for R_g, t_g, R_c, t_c in zip(all_R_base_to_gripper, all_t_base_to_gripper, all_R_cam_to_target, all_t_cam_to_target):
            projected_t_c = np.dot(R_g, self.t_cam_to_base.reshape(3, 1)).flatten() + t_g.flatten()
            error = np.linalg.norm(t_c.flatten() - projected_t_c)
            errors.append(error)

        mean_error = np.mean(errors)
        stdev_error = np.std(errors)
        return mean_error, stdev_error
   
    
    def calculate_3D_error(self, all_R_base_to_gripper, all_t_base_to_gripper, all_R_cam_to_target, all_t_cam_to_target):
        errors_3d = []
        errors_angular = []
        
        rotation_C_to_B = self.R_cam_to_base  # rotation de C vers B
        translation_C_to_B = self.t_cam_to_base  # translation de C vers B

        
        rotation_matrix_C_to_B = R.from_euler('ZYX', np.flip(rotation_C_to_B)).as_matrix()

        # Construire la matrice de transformation du repère C vers B
        transformation_matrix_C_to_B = np.eye(4)
        transformation_matrix_C_to_B[:3, :3] = rotation_matrix_C_to_B
        transformation_matrix_C_to_B[:3, 3] = translation_C_to_B
        print("transformation tx ty tz rpy deg :",getTransformasTxTyTzRPYdeg(transformation_matrix_C_to_B))
        for rotation_A_to_B_euler, translation_A_to_B, rotation_A_to_C_euler, translation_A_to_C in zip(all_R_base_to_gripper, all_t_base_to_gripper, all_R_cam_to_target, all_t_cam_to_target):
            reprojection_error, angular_error = calculate_3d_reprojection_and_angular_error(translation_A_to_C, rotation_A_to_C_euler, 
                                                                                            translation_A_to_B, rotation_A_to_B_euler,
                                                                                            transformation_matrix_C_to_B)
            errors_3d.append(reprojection_error)
            print("errors_3d",reprojection_error)
            errors_angular.append(angular_error)
        #print("errors_3d",errors_3d)
            
        mean_3D_error = np.mean(errors_3d)
        stdev_3D_error = np.std(errors_3d)
    
        mean_angular_error = np.mean(errors_angular)
        stdev_angular_error = np.std(errors_angular)
        
        return mean_3D_error, stdev_3D_error, mean_angular_error, stdev_angular_error

    def create_homogeneous_matrix(self, R, t):
        """
        Crée une matrice homogène 4x4 à partir d'une matrice de rotation et d'un vecteur de translation.
        """
        H = np.eye(4)
        H[:3, :3] = R  # Rotation 3x3
        H[:3, 3] = t.flatten()  # Translation 3x1
        return m3d.Transform(H)


    def get_eye_to_hand(self, indices, write_to_csv=False, print_results=False):
        all_R_cam_to_target, all_t_cam_to_target, all_R_base_to_gripper, all_t_base_to_gripper = [], [], [], []

        for idx in indices:
            pose_data = self.merged_poses[idx]
            R_base_to_gripper, t_base_to_gripper = get_rot_trans(pose_data['gripper_pose'])
            R_cam_to_target, t_cam_to_target = get_rot_trans(pose_data['tag_pose'])

            all_R_cam_to_target.append(R_cam_to_target)
            all_t_cam_to_target.append(t_cam_to_target)
            all_R_base_to_gripper.append(R_base_to_gripper)
            all_t_base_to_gripper.append(t_base_to_gripper)

        # Convertir en matrices homogènes 4x4
        pose_pairs = []
        for Rg, tg, Rc, tc in zip(all_R_base_to_gripper, all_t_base_to_gripper, all_R_cam_to_target, all_t_cam_to_target):
            pose_base_to_gripper = self.create_homogeneous_matrix(Rg, tg)
            pose_cam_to_target = self.create_homogeneous_matrix(Rc, tc)
            pose_pairs.append((pose_base_to_gripper, pose_cam_to_target))

        # Debugging - Print Aruco in camera and base
        print("Aruco in camera", getTransformasTxTyTzRPYdeg2(all_R_cam_to_target[0], all_t_cam_to_target[0]))
        print("Aruco in base", getTransformasTxTyTzRPYdeg2(all_R_base_to_gripper[0], all_t_base_to_gripper[0]))


        # Appeler la calibration Tsai-Lenz avec les paires de transformations homogènes
        tsai_lenz_calibrator = TsaiHandEyeCalibration(pose_pairs)
        self.R_cam_to_base = tsai_lenz_calibrator.sensor_in_flange.orient.rotation_matrix
        self.t_cam_to_base = tsai_lenz_calibrator.sensor_in_flange.pos.data

        # Calculer l'erreur 3D et l'erreur angulaire
        mean_3D_error, stdev_3D_error, _, _ = self.calculate_3D_error(all_R_base_to_gripper, all_t_base_to_gripper, all_R_cam_to_target, all_t_cam_to_target)

        # Affichage des résultats si demandé
        if print_results:
            print_get_eye_to_hand(indices, self.R_cam_to_base, self.t_cam_to_base, mean_3D_error, stdev_3D_error)
        
        # Écriture dans un fichier CSV si demandé
        if write_to_csv:
            write_to_csv(self.output_dir, len(indices), self.arm, self.R_cam_to_base, self.t_cam_to_base, mean_3D_error, stdev_3D_error)


    def get_eye_to_hand3(self, indices, write_to_csv=False,print_results=False):
        all_R_cam_to_target, all_t_cam_to_target, all_R_base_to_gripper, all_t_base_to_gripper = [], [], [], []

        for idx in indices:
            pose_data = self.merged_poses[idx]
            R_base_to_gripper, t_base_to_gripper = get_rot_trans(pose_data['gripper_pose'])
            R_cam_to_target, t_cam_to_target = get_rot_trans(pose_data['tag_pose'])
            
            all_R_cam_to_target.append(R_cam_to_target)
            all_t_cam_to_target.append(t_cam_to_target)
            all_R_base_to_gripper.append(R_base_to_gripper)
            all_t_base_to_gripper.append(t_base_to_gripper)

        all_R_cam_to_target = np.array(all_R_cam_to_target)
        all_t_cam_to_target = np.array(all_t_cam_to_target)
        all_R_base_to_gripper = np.array(all_R_base_to_gripper)
        all_t_base_to_gripper = np.array(all_t_base_to_gripper)
        
        print("Aruco in camera",getTransformasTxTyTzRPYdeg2(all_R_cam_to_target[0],all_t_cam_to_target[0]))
        print("aruca in base",getTransformasTxTyTzRPYdeg2(all_R_base_to_gripper[0],all_t_base_to_gripper[0]))
        print("all_R_cam_to_target",all_R_cam_to_target)
        print("all_t_cam_to_target",all_t_cam_to_target)

        all_R_cam_to_target[0]=np.eye(3)
        all_t_cam_to_target[0]=np.zeros((3,1))
        all_t_cam_to_target[0][0]=1.0
        print("all_R_cam_to_target",all_R_cam_to_target)
        print("all_t_cam_to_target",all_t_cam_to_target)
        all_R_base_to_gripper[0]=np.eye(3)
        all_t_base_to_gripper[0]=np.zeros((3,1))
        all_t_base_to_gripper[0][0]=15.0
        print("all_t_cam_to_target",all_t_cam_to_target)
        print("all_t_base_to_gripper",all_t_base_to_gripper)

        
        self.R_cam_to_base, self.t_cam_to_base = TsaiHandEyeCalibration(
            all_R_base_to_gripper, all_t_base_to_gripper, 
            all_R_cam_to_target, all_t_cam_to_target
        )
        self.R_cam_to_base = np.flip(R.from_matrix(self.R_cam_to_base).as_euler('ZYX', degrees=False))

        mean_3D_error, stdev_3D_error , _, _= self.calculate_3D_error(all_R_base_to_gripper, all_t_base_to_gripper, all_R_cam_to_target, all_t_cam_to_target)
        #mean_3D_error, stdev_3D_error= self.calculate_3D_error(all_R_base_to_gripper, all_t_base_to_gripper, all_R_cam_to_target, all_t_cam_to_target)
        
        if print_results:
            print_get_eye_to_hand(indices,self.R_cam_to_base,self.t_cam_to_base,mean_3D_error,stdev_3D_error)
        if write_to_csv:
            write_to_csv(self.output_dir, len(indices), self.arm, self.R_cam_to_base, self.t_cam_to_base, mean_3D_error, stdev_3D_error)

    def get_eye_to_hand_with_barycenter(self, indices, write_to_csv=False, print_results=False):
        """
        Effectue la calibration en utilisant les barycentres des poses fournies.
        """
        gripper_barycenter, tag_barycenter = self.calculate_barycenter(self.merged_poses,indices)

        R_base_to_gripper, t_base_to_gripper = get_rot_trans(gripper_barycenter)
        R_cam_to_target, t_cam_to_target = get_rot_trans(tag_barycenter)

        self.R_cam_to_base, self.t_cam_to_base = TsaiHandEyeCalibration(
            np.array([R_base_to_gripper]), np.array([t_base_to_gripper]), 
            np.array([R_cam_to_target]), np.array([t_cam_to_target])
        )
        self.R_cam_to_base = R.from_matrix(self.R_cam_to_base).as_euler('xyz', degrees=False)

        mean_3D_error, stdev_3D_error , _, _= self.calculate_3D_error(
            [R_base_to_gripper], [t_base_to_gripper], [R_cam_to_target], [t_cam_to_target])
        
        if print_results:
            print_get_eye_to_hand(indices,self.R_cam_to_base,self.t_cam_to_base,mean_3D_error,stdev_3D_error)
        if write_to_csv:
            write_to_csv(self.output_dir, len(indices), self.arm, self.R_cam_to_base, self.t_cam_to_base, mean_3D_error, stdev_3D_error)
            
    def get_eye_to_hand_progressively(self):
        if os.path.exists(self.output_dir):
            os.remove(self.output_dir)
        
        pose_shuffled = list(range(len(self.merged_poses)))
        random.shuffle(pose_shuffled)

        for i in range(1, len(self.merged_poses) + 1):
            current_indices = pose_shuffled[:i]
            self.get_eye_to_hand(current_indices, print_results=True,write_to_csv=True)

    def get_eye_to_hand_progressively_from_sequence(self, sequence):
        if os.path.exists(self.output_dir):
            os.remove(self.output_dir)
        
        for i in range(1, len(sequence) + 1):
            current_indices = sequence[:i]
            self.get_eye_to_hand(current_indices, write_to_csv=True,print_results=True)
    
    def find_inliers_combinations(self, error_threshold, type_error, precision_threshold=None):
        current_indices = []

        for pose_id in range(len(self.merged_poses)):
            best_mean_error = float('inf')
            best_new_index = None
            
            for idx in range(len(self.merged_poses)):
                if idx not in current_indices:
                    trial_indices = current_indices + [idx]
                    
                    self.get_eye_to_hand(trial_indices)

                    all_R_base_to_gripper = [extract_rot_matrix(self.merged_poses[i]['gripper_pose']) for i in trial_indices]
                    all_t_base_to_gripper = [extract_translation(self.merged_poses[i]['gripper_pose']).reshape(3, 1) for i in trial_indices]
                    all_R_cam_to_target = [extract_rot_matrix(self.merged_poses[i]['tag_pose']) for i in trial_indices]
                    all_t_cam_to_target = [extract_translation(self.merged_poses[i]['tag_pose']).reshape(3, 1) for i in trial_indices]

                    if type_error == '3D_error':
                        mean_error, _, _, _ = self.calculate_3D_error(all_R_base_to_gripper, all_t_base_to_gripper, all_R_cam_to_target, all_t_cam_to_target)
                        #mean_error, _,= self.calculate_3D_error(all_R_base_to_gripper, all_t_base_to_gripper, all_R_cam_to_target, all_t_cam_to_target)

                    else:
                        mean_error, _ = self.calculate_reprojection_error(all_R_base_to_gripper, all_t_base_to_gripper, all_R_cam_to_target, all_t_cam_to_target)
                    
                    if mean_error <= error_threshold and mean_error < best_mean_error:
                        #print(f"============Found inliers combination with {len(trial_indices)} images=======")
                        #print(f"Mean error: {mean_error}")
                        #print(f"Best mean error: {best_mean_error}")
                        best_mean_error = mean_error
                        best_new_index = idx
            
            print(f"Best mean error found for pose {pose_id}: {best_mean_error}")
            if best_new_index is not None:
                current_indices.append(best_new_index)
                self.best_combinations.append((len(current_indices), current_indices.copy()))

                if precision_threshold is not None and best_mean_error <= precision_threshold:
                    print(f"Precision threshold of {precision_threshold} reached with mean error {best_mean_error}. Stopping search.")
                    break
            else:
                break

    
    def find_inliers_combinations_ransac(self, nb_per_trial, nb_max_trial, type_error='3D_error'):
        """
        Utilise une approche RANSAC pour trouver les meilleures combinaisons de poses en fonction de l'erreur minimale,
        en utilisant les barycentres des poses.
        """
        best_combinations = []

        for num_samples in nb_per_trial:
            best_mean_error = float('inf')
            best_indices = None

            for _ in range(nb_max_trial):
                trial_indices = np.random.choice(len(self.merged_poses), num_samples, replace=False)

                #self.get_eye_to_hand_with_barycenter(trial_indices, write_to_csv=False, print_results=False)
                self.get_eye_to_hand(trial_indices, write_to_csv=False, print_results=False)

                all_R_base_to_gripper = [self.extract_rot_matrix(self.merged_poses[i]['gripper_pose']) for i in trial_indices]
                all_t_base_to_gripper = [self.extract_translation(self.merged_poses[i]['gripper_pose']).reshape(3, 1) for i in trial_indices]
                all_R_cam_to_target = [self.extract_rot_matrix(self.merged_poses[i]['tag_pose']) for i in trial_indices]
                all_t_cam_to_target = [self.extract_translation(self.merged_poses[i]['tag_pose']).reshape(3, 1) for i in trial_indices]

                if type_error == '3D_error':
                    mean_error, _ , _, _= self.calculate_3D_error(all_R_base_to_gripper, all_t_base_to_gripper, all_R_cam_to_target, all_t_cam_to_target)
                else:
                    mean_error, _ = self.calculate_reprojection_error(all_R_base_to_gripper, all_t_base_to_gripper, all_R_cam_to_target, all_t_cam_to_target)

                if mean_error < best_mean_error:
                    best_mean_error = mean_error
                    best_indices = trial_indices

            if best_indices is not None:
                best_combinations.append((num_samples, best_indices, best_mean_error))

        print("Best combinations found with RANSAC using barycenters:")
        for num_samples, indices, mean_error in best_combinations:
            print(f"Sample size: {num_samples}, Indices: {indices}, Mean error: {mean_error}")

        self.best_combinations = best_combinations

output_image_path = 'calib_results.csv'
eye_in_hand_calib = EyeInHandCalib("merged_poses.csv", output_image_path, "Right")
compute_inliers = True
if compute_inliers:
    eye_in_hand_calib.find_inliers_combinations(error_threshold=1,type_error='3D_error',)
    #eye_in_hand_calib.find_inliers_combinations_ransac(nb_per_trial=[2, 5, 10, 15, 20,30,40,50,75], nb_max_trial=10000, type_error='3D_error')
    print("Best combinations found:", eye_in_hand_calib.best_combinations)
    if eye_in_hand_calib.best_combinations:
        print("Inliers combinations:", eye_in_hand_calib.best_combinations[-1])
        output_best_combinations = eye_in_hand_calib.best_combinations[-1]
        best_combination = output_best_combinations[1]

    else:
        print("No valid inliers combination found. Try increasing the error_threshold or checking the data.")
        best_combination = []
    print ("Best combination: ", best_combination)

#output_best_combinations = (142, [461, 462, 326, 460, 463, 327, 459, 325, 293, 400, 425, 292, 365, 51, 257, 366, 367, 256, 364, 363, 258, 401, 50, 49, 52, 370, 291, 369, 328, 368, 97, 371, 413, 402, 53, 399, 255, 209, 324, 362, 112, 208, 372, 79, 187, 80, 230, 412, 323, 424, 113, 188, 411, 78, 210, 322, 27, 426, 290, 28, 321, 2, 3, 26, 17, 1, 189, 77, 20, 22, 21, 186, 25, 19, 18, 24, 23, 16, 414, 111, 458, 289, 4, 7, 0, 11, 8, 13, 9, 10, 15, 14, 12, 6, 417, 114, 464, 81, 5, 254, 231, 320, 29, 207, 98, 211, 54, 303, 410, 110, 477, 288, 304, 305, 329, 418, 76, 478, 109, 306, 427, 373, 307, 308, 403, 479, 82, 480, 108, 48, 428, 115, 190, 272, 229, 107, 476, 273, 457, 294, 274, 475])
#output_best_combinations = (23, [461, 462, 326, 460, 463, 327, 459, 325, 293, 400, 425, 292, 365, 51, 257, 366, 367, 256, 364, 363, 258, 401, 50])

#eye_in_hand_calib.get_eye_to_hand(best_combination,write_to_csv=False, print_results=True)
eye_in_hand_calib.get_eye_to_hand_progressively_from_sequence(best_combination)
#eye_in_hand_calib.get_eye_to_hand_progressively()
