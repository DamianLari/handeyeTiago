#!/usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
import os
import cv2
from cv_bridge import CvBridge
import csv
import json
import argparse
import shutil
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation as R

class RecordFromTopic:
    def __init__(self, image_topic, camera_info_topic, tf_topic, outputCSV, image_rate, timeout_duration, output_dir, bag_file=None):
        self.image_topic = image_topic
        self.camera_info_topic = camera_info_topic
        self.tf_topic = tf_topic
        self.aruco_pose_topic = "/aruco_pose"
        self.image_rate = image_rate
        self.timeout_duration = timeout_duration
        self.bridge = CvBridge()
        self.output_dir = output_dir
        self.last_msg_time = rospy.Time.now()
        
        self.image_counter = 0
        self.save_counter = 0
        self.pose_counter = 0
        self.csv_filename = outputCSV

        self.bag_file = bag_file
       
        self.cleanup()

        if not os.path.isfile(self.csv_filename):
            with open(self.csv_filename, 'w', newline='') as csvfile:
                fieldnames = ['tag_tx', 'tag_ty', 'tag_tz', 'tag_rx', 'tag_ry', 'tag_rz',
                'gripper_tx', 'gripper_ty', 'gripper_tz', 'gripper_rx', 'gripper_ry', 'gripper_rz',
                'head_tx', 'head_ty', 'head_tz', 'head_rx', 'head_ry', 'head_rz',]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.fetch_data()

    def cleanup(self):
        if os.path.exists(self.output_dir):
            shutil.rmtree(self.output_dir)
        os.makedirs(self.output_dir)

        if os.path.exists(self.csv_filename):
            os.remove(self.csv_filename)

    def tf_listener(self, tf_frame, tf_child):
        try:
            transform = self.tf_buffer.lookup_transform(tf_frame, tf_child, rospy.Time(0), rospy.Duration(10.0))
            pose = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            rota = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            rotation = R.from_quat(rota).as_euler("ZYX")
            pose.extend(rotation[::-1])
            return pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transformation non trouvée : {e}")
            return None

    def fetch_data(self):
        rate = rospy.Rate(2)  # Fréquence de 2 Hz
        while not rospy.is_shutdown():
            tag_pose = self.tf_listener('xtion_rgb_optical_frame', 'aruco_frame')
            gripper_pose = self.tf_listener('base_link', 'aruco_frame')
            head_pose = self.tf_listener('base_link', 'xtion_rgb_optical_frame')
            self.record_pose(tag_pose, gripper_pose,head_pose)
            rate.sleep()
            
    def record_pose(self,tag_pose,gripper_pose,head_pose):
        if tag_pose is None or gripper_pose is None:
            rospy.logwarn("Une des transformations est manquante. Enregistrement ignoré.")
            return
        try:
            with open(self.csv_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    tag_pose[0], tag_pose[1], tag_pose[2],  
                    tag_pose[3], tag_pose[4], tag_pose[5],
                    gripper_pose[0], gripper_pose[1], gripper_pose[2],  
                    gripper_pose[3], gripper_pose[4], gripper_pose[5],
                    head_pose[0], head_pose[1], head_pose[2],  
                    head_pose[3], head_pose[4], head_pose[5]  
                ])
                self.pose_counter += 1
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(f"Transformation non trouvée : {e}")

parser = argparse.ArgumentParser(description='Enregistrement des images et des poses du gripper à partir des topics de la caméra')
parser.add_argument('--image_topic', type=str, default='/xtion/rgb/image', help='Le topic pour les images de la caméra')
parser.add_argument('--camera_info_topic', type=str, default='/xtion/rgb/camera_info', help='Le topic pour les messages d\'info de la caméra')
parser.add_argument('--tf_topic', type=str, default='/tf', help='Le topic pour les transformations TF')
parser.add_argument('--image_rate', type=int, default=10, help='Sauvegarder une image sur x images')
parser.add_argument('--timeout', type=float, default=2.0, help='Temps d\'attente de message avant arrêt du programme')
parser.add_argument('--gripper_pose_csv', type=str, default='gripper_pose.csv', help='Fichier CSV où sauvegarder les poses du gripper')
parser.add_argument('--output_dir', type=str, default='images', help='Dossier où sauvegarder les images')
parser.add_argument('--bag_file', type=str, help='Fichier rosbag à lire')

args = parser.parse_args()

rospy.init_node('topic_recorder', anonymous=True)

recorder = RecordFromTopic(args.image_topic, args.camera_info_topic, args.tf_topic, args.gripper_pose_csv, args.image_rate, args.timeout, args.output_dir, args.bag_file)

if not args.bag_file:
    rospy.spin()
