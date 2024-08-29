#!/bin/bash
#python3 HandEyeCalib.py --merged_poses_csv  merged_poses.csv

python3 RecordFromTopic.py --image_topic /xtion/rgb/image --camera_info_topic /xtion/rgb/camera_info --gripper_pose_csv gripper_pose.csv --image_rate 5 --timeout 2.0
python3 GetTagPose.py --image_folder images --calibration_file calib_data.json --tag_id 11 --csv_filename tag_pose.csv
python3 MergePose.py --gripper_pose_csv gripper_pose.csv --tag_pose_csv tag_pose.csv --output_csv merged_poses.csv
python3 calibExtra.py

python3 RecordFromTopic.py --image_topic /xtion/rgb/image --camera_info_topic /xtion/rgb/camera_info --gripper_pose_csv gripper_pose.csv --image_rate 3 --timeout 2.0
python3 GetTagPose.py --image_folder images --calibration_file calib_data.json --tag_id 11 --csv_filename tag_pose.csv
python3 MergePose.py --gripper_pose_csv gripper_pose.csv --tag_pose_csv tag_pose.csv --output_csv merged_poses.csv
python3 calibExtra.py

python3 RecordFromTopic.py --image_topic /xtion/rgb/image --camera_info_topic /xtion/rgb/camera_info --gripper_pose_csv gripper_pose.csv --image_rate 1 --timeout 2.0
python3 GetTagPose.py --image_folder images --calibration_file calib_data.json --tag_id 11 --csv_filename tag_pose.csv
python3 MergePose.py --gripper_pose_csv gripper_pose.csv --tag_pose_csv tag_pose.csv --output_csv merged_poses.csv
python3 calibExtra.py



python3 RecordFromTopic.py --image_topic /xtion/rgb/image --camera_info_topic /xtion/rgb/camera_info --gripper_pose_csv gripper_pose.csv --image_rate 5 --timeout 2.0 
python3 GetTagPose.py --image_folder images --calibration_file calib_data.json --tag_id 11 --csv_filename tag_pose.csv
python3 MergePose.py --gripper_pose_csv gripper_pose.csv --tag_pose_csv tag_pose.csv --output_csv merged_poses.csv
python3 calibExtra.py

python3 RecordFromTopic.py --image_topic /xtion/rgb/image --camera_info_topic /xtion/rgb/camera_info --gripper_pose_csv gripper_pose.csv --image_rate 3 --timeout 2.0 
python3 GetTagPose.py --image_folder images --calibration_file calib_data.json --tag_id 11 --csv_filename tag_pose.csv
python3 MergePose.py --gripper_pose_csv gripper_pose.csv --tag_pose_csv tag_pose.csv --output_csv merged_poses.csv
python3 calibExtra.py

python3 RecordFromTopic.py --image_topic /xtion/rgb/image --camera_info_topic /xtion/rgb/camera_info --gripper_pose_csv gripper_pose.csv --image_rate 1 --timeout 2.0 
python3 GetTagPose.py --image_folder images --calibration_file calib_data.json --tag_id 11 --csv_filename tag_pose.csv
python3 MergePose.py --gripper_pose_csv gripper_pose.csv --tag_pose_csv tag_pose.csv --output_csv merged_poses.csv
python3 calibExtra.py