import csv
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import matplotlib
import random
matplotlib.use('Agg')
import pandas as pd
import seaborn as sns


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
            estimated_heads.append(estimated_head[:3, 3]) 
            real_heads.append(head_matrices[i][:3, 3]) 

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
            sample_indices = random.sample(range(len(R_base_cam)), 3)
            #sampled_R = [R_base_cam[i] for i in sample_indices]
            sampled_R_YPR=[np.flip(R_base_cam[i]) for i in sample_indices]
            sampled_t = [t_base_cam[i] for i in sample_indices]

            #avg_R = np.mean(sampled_R, axis=0)
            avg_R = np.flip(sampled_R_YPR.mean().as_euler('ZYX',degrees=False ))
            
            avg_t = np.mean(sampled_t, axis=0)

            inliers_count = 0
            for i in range(len(R_base_cam)):
                diff_R = np.linalg.norm(R_base_cam[i] - avg_R)
                diff_t = np.linalg.norm(t_base_cam[i] - avg_t)
                if diff_R < threshold and diff_t < threshold:
                    inliers_count += 1

            if inliers_count > best_inliers_count:
                best_inliers_count = inliers_count
                best_R = avg_R
                best_t = avg_t

        return np.flip(R.from_matrix(best_R).as_euler('ZYX')), best_t
    
    def generate_final_comparison(self, best_R, best_t):
        mean_real_t = np.mean(self.real_heads, axis=0)
        mean_real_R = np.mean([R.from_matrix(T[:3, :3]).as_euler('ZYX') for T in self.convert_to_transform_matrices(self.head_positions)], axis=0)

        ransac_t = np.array(best_t)
        ransac_R = np.array(best_R)

        diff_t = mean_real_t - ransac_t
        diff_R = mean_real_R - ransac_R

        data = {
            'Vérité terrain (Translation)': mean_real_t,
            'RANSAC (Translation)': ransac_t,
            'Différence (Translation)': diff_t,
            'Vérité terrain (Rotation)': mean_real_R,
            'RANSAC (Rotation)': ransac_R,
            'Différence (Rotation)': diff_R
        }

        df = pd.DataFrame(data, index=['X', 'Y', 'Z'])
        df.to_csv('final_comparison_table.csv')
        print(df)

        fig, ax = plt.subplots(figsize=(16, 3))
        ax.axis('off')
        table = ax.table(cellText=df.values, colLabels=df.columns, rowLabels=df.index, cellLoc='center', loc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(14)
        table.scale(1.5, 1.8)  
        plt.title('Comparaison Finale des Valeurs de Translation et de Rotation')
        plt.savefig('final_comparison_table.png', dpi=300, bbox_inches='tight')
        plt.close()





if __name__ == "__main__":
    csv_filename = "handeye.csv"
    handeye = HandEyeCalibration(csv_filename)
    handeye.calculate_estimated_heads()
    handeye.plot_comparison()
    best_R, best_t = handeye.ransac_for_base_to_camera()
    print("Meilleure estimation de R (Base -> Caméra):\n", best_R)
    print("Meilleure estimation de t (Base -> Caméra):\n", best_t)
    handeye.generate_final_comparison(best_R,best_t)