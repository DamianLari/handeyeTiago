import csv
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import matplotlib
import random
matplotlib.use('Agg')


class HandEyeCalibration:
    def __init__(self, csv_filename):
        self.csv_filename = csv_filename
        self.CamTag = []
        self.BaseTag = []
        self.head_positions = []
        self.estimated_heads = None
        self.real_heads = None
        self.load_data_from_csv()

    def load_data_from_csv(self):
        with open(self.csv_filename, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                tag_pose = [
                    float(row['tag_tx']), float(row['tag_ty']), float(row['tag_tz']),
                    float(row['tag_rx']), float(row['tag_ry']), float(row['tag_rz'])
                ]
                gripper_pose = [
                    float(row['gripper_tx']), float(row['gripper_ty']), float(row['gripper_tz']),
                    float(row['gripper_rx']), float(row['gripper_ry']), float(row['gripper_rz'])
                ]
                head_pose = [
                    float(row['head_tx']), float(row['head_ty']), float(row['head_tz']),
                    float(row['head_rx']), float(row['head_ry']), float(row['head_rz'])
                ]
                self.CamTag.append(tag_pose)
                self.BaseTag.append(gripper_pose)
                self.head_positions.append(head_pose)

    def convert_to_transform_matrices(self, positions):
        transform_matrices = []
        for pose in positions:
            translation = np.array(pose[:3])
            rotation = R.from_euler('ZYX', [pose[5], pose[4], pose[3]]).as_matrix()
            transform = np.eye(4)
            transform[:3, :3] = rotation
            transform[:3, 3] = translation
            transform_matrices.append(transform)
        return transform_matrices

    def calculate_base_to_camera_transform(self):
        all_T_cam_tag = self.convert_to_transform_matrices(self.CamTag)
        all_T_base_tag = self.convert_to_transform_matrices(self.BaseTag)
        
        all_T_base_cam = []

        for T_cam_tag, T_base_tag in zip(all_T_cam_tag, all_T_base_tag):
            T_base_cam = T_base_tag.dot(np.linalg.inv(T_cam_tag))
            all_T_base_cam.append(T_base_cam)

        all_R_base_cam = [T[:3, :3] for T in all_T_base_cam]
        all_t_base_cam = [T[:3, 3] for T in all_T_base_cam]
        
        return all_R_base_cam, all_t_base_cam

    def calculate_estimated_heads(self):
        # Calculer la transformation base -> caméra
        R_base_cam, t_base_cam = self.calculate_base_to_camera_transform()

        gripper_matrices = self.convert_to_transform_matrices(self.BaseTag)
        head_matrices = self.convert_to_transform_matrices(self.head_positions)

        estimated_heads = []
        real_heads = []

        for i, gripper_mat in enumerate(gripper_matrices):
            base_to_cam_transform = np.eye(4)
            base_to_cam_transform[:3, :3] = R_base_cam[i]
            base_to_cam_transform[:3, 3] = t_base_cam[i]

            estimated_head = base_to_cam_transform
            estimated_heads.append(estimated_head[:3, 3])  # Only translation part
            real_heads.append(head_matrices[i][:3, 3])  # Only translation part

        self.estimated_heads = np.array(estimated_heads)
        self.real_heads = np.array(real_heads)
    
    def plot_comparison(self):
        if self.estimated_heads is None or self.real_heads is None:
            raise ValueError("Les données estimées et réelles ne sont pas disponibles. Veuillez exécuter 'calculate_estimated_heads()' d'abord.")

        fig, axs = plt.subplots(3, 1, figsize=(10, 15))
        labels = ['X', 'Y', 'Z']

        for i in range(3):
            axs[i].plot(self.estimated_heads[:, i], label='Estimé', linestyle='--')
            axs[i].plot(self.real_heads[:, i], label='Réel', linestyle='-')
            axs[i].set_title(f'Comparaison de la position {labels[i]}')
            axs[i].set_xlabel('Échantillon')
            axs[i].set_ylabel(f'Position {labels[i]} (m)')
            axs[i].legend()
            axs[i].grid()

        plt.tight_layout()
        plt.savefig('handeyeresults.png')

    def ransac_for_base_to_camera(self, iterations=100, threshold=0.05):
        R_base_cam, t_base_cam = self.calculate_base_to_camera_transform()
        best_inliers_count = 0
        best_R = None
        best_t = None

        for _ in range(iterations):
            # Sélectionner un échantillon aléatoire
            sample_indices = random.sample(range(len(R_base_cam)), 3)
            sampled_R = [R_base_cam[i] for i in sample_indices]
            sampled_t = [t_base_cam[i] for i in sample_indices]

            # Calculer une estimation moyenne
            avg_R = np.mean(sampled_R, axis=0)
            avg_t = np.mean(sampled_t, axis=0)

            # Compter les inliers
            inliers_count = 0
            for i in range(len(R_base_cam)):
                diff_R = np.linalg.norm(R_base_cam[i] - avg_R)
                diff_t = np.linalg.norm(t_base_cam[i] - avg_t)
                if diff_R < threshold and diff_t < threshold:
                    inliers_count += 1

            # Mettre à jour si on trouve une meilleure estimation
            if inliers_count > best_inliers_count:
                best_inliers_count = inliers_count
                best_R = avg_R
                best_t = avg_t

        return np.flip(R.from_matrix(best_R).as_euler('ZYX')), best_t

if __name__ == "__main__":
    csv_filename = "handeye.csv"  # Remplacez par le chemin vers votre fichier CSV
    handeye = HandEyeCalibration(csv_filename)
    handeye.calculate_estimated_heads()
    handeye.plot_comparison()
    best_R, best_t = handeye.ransac_for_base_to_camera()
    print("Meilleure estimation de R (Base -> Caméra):\n", best_R)
    print("Meilleure estimation de t (Base -> Caméra):\n", best_t)