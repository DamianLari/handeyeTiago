from MergePose import MergePose
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2
import argparse

class HandToEyeCalib():
    def __init__(self, MergedPose,Pose1FilePath,Pose2FilePath):
        print(MergedPose)
        merger = MergePose(Pose1FilePath, Pose2FilePath)
        if MergedPose is None:
            self.merged_poses = merger.merge_poses()
        else:
            self.merged_poses = merger.read_merged_poses(MergedPose)
        
    def extract_rot_matrix(self,pose):
        return R.from_euler('XYZ', [pose['rx'], pose['ry'], pose['rz']], degrees=False).as_matrix()
                
    
    def extract_translation(self,pose):
        return np.array([pose['tx'], pose['ty'], pose['tz']])

    def get_hand_to_eye(self):
        all_R_target_to_cam = []
        all_t_target_to_cam = []
        all_R_gripper_to_base = []
        all_t_gripper_to_base = []
        
        for image__file in self.merged_poses:
            tag_pose_from_camera = self.merged_poses[image__file]['tag_pose']
            gripper_pose_from_base = self.merged_poses[image__file]['gripper_pose']
            
            if tag_pose_from_camera != [0,0,0,0,0,0]:
                R_gripper_to_base = self.extract_rot_matrix(gripper_pose_from_base)
                t_gripper_to_base = self.extract_translation(gripper_pose_from_base).reshape(3,1)
               
                R_target_to_cam = self.extract_rot_matrix(tag_pose_from_camera)
                t_target_to_cam = self.extract_translation(tag_pose_from_camera).reshape(3,1)
                
                all_R_target_to_cam.append(R_target_to_cam)
                all_t_target_to_cam.append(t_target_to_cam)
                
                all_R_gripper_to_base.append(R_gripper_to_base)
                all_t_gripper_to_base.append(t_gripper_to_base)
                
  
                
        self.R_cam_to_gripper, self.t_cam_to_gripper = cv2.calibrateHandEye(all_R_gripper_to_base, all_t_gripper_to_base, all_R_target_to_cam, all_t_target_to_cam, cv2.CALIB_HAND_EYE_PARK)
        print(self.R_cam_to_gripper,self.t_cam_to_gripper)
       

    

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

    def update_visualizer(self):
        self.open3d_toolbox.updateRenderer()


parser = argparse.ArgumentParser(description='Estimation de la pose d\'un tag et enregistrement dans un fichier CSV')
parser.add_argument('--merged_poses_csv', type=str, default=None, help='Chemin du fichier CSV contenant les poses fusionnées')
parser.add_argument('--gripper_pose_csv', type=str, help='Chemin du fichier CSV contenant les poses du gripper')
parser.add_argument('--tag_pose_csv', type=str, help='Chemin du fichier CSV contenant les poses du tag')
args = parser.parse_args()

do_hand_eye_calib = HandToEyeCalib(args.merged_poses_csv,args.gripper_pose_csv, args.tag_pose_csv)
do_hand_eye_calib.get_hand_to_eye()

