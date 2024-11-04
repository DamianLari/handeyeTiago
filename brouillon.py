def get_eye_to_hand(self, indices,write_to_csv=False):
        """
        Effectue la calibration en utilisant uniquement les indices fournis.
        """
        all_R_cam_to_target, all_t_cam_to_target, all_R_base_to_gripper, all_t_base_to_gripper = [], [], [], []

        for idx in indices:
            pose_data = self.merged_poses[idx]
            tag_pose_from_camera = pose_data['tag_pose']
            gripper_pose_from_base = pose_data['gripper_pose']
            
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

        self.R_cam_to_base, self.t_cam_to_base = TsaiHandEyeCalibration(
            all_R_base_to_gripper, all_t_base_to_gripper, 
            all_R_cam_to_target, all_t_cam_to_target
        )

        self.R_cam_to_base = R.from_matrix(self.R_cam_to_base).as_euler('xyz', degrees=False)
        mean_error, stdev_error = self.calculate_reprojection_error(
            all_R_base_to_gripper, all_t_base_to_gripper, all_R_cam_to_target, all_t_cam_to_target)
        
        mean_3D_error, stdev_3D_error = self.calculate_3D_error(
            all_R_base_to_gripper, all_t_base_to_gripper, all_R_cam_to_target, all_t_cam_to_target)
        
        print(f"Calibration with provided inlier indices:")
        print("  Rotation matrix from camera to base:", self.R_cam_to_base)
        print("  Translation vector from camera to base:", self.t_cam_to_base)
        print("  Mean reprojection error:", mean_error)
        print("  Standard deviation of reprojection error:", stdev_error)
        print("  Mean 3D positional error:", mean_3D_error)
        print("  Standard deviation of 3D positional error:", stdev_3D_error)
        if write_to_csv:
            self.write_to_csv(len(indices), mean_error, stdev_error, mean_3D_error, stdev_3D_error)