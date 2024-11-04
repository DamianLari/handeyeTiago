#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
import os
import cv2
from cv_bridge import CvBridge
import csv
import json
import argparse
import shutil
import numpy as np

class RecordFromTopic:
    def __init__(self, image_topic, camera_info_topic, aruco_pose_topic, outputCSV, image_rate, timeout_duration, output_dir, bag_file=None):
        self.image_topic = image_topic
        self.camera_info_topic = camera_info_topic
        self.aruco_pose_topic = aruco_pose_topic  # Nouveau topic pour les poses
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
                fieldnames = ['time', 'tx', 'ty', 'tz']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()

        # Souscription au topic de l'image et de la caméra
        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        rospy.Subscriber(self.aruco_pose_topic, Float32MultiArray, self.aruco_pose_callback)  # Souscrire au topic des poses
        rospy.Timer(rospy.Duration(1), self.check_for_timeout)

    def cleanup(self):
        if os.path.exists(self.output_dir):
            shutil.rmtree(self.output_dir)
        os.makedirs(self.output_dir)

        if os.path.exists(self.csv_filename):
            os.remove(self.csv_filename)

    def image_callback(self, msg):
        self.last_msg_time = rospy.Time.now()
        self.save_counter += 1
        if self.save_counter % self.image_rate == 0:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            filename = f'{self.output_dir}/image_{self.image_counter:04d}.jpg'
            cv2.imwrite(filename, cv_image)
            self.image_counter += 1

    def camera_info_callback(self, msg):
        self.last_msg_time = rospy.Time.now()
        camera_info = {
            'mtx': [
                [msg.K[0], msg.K[1], msg.K[2]],
                [msg.K[3], msg.K[4], msg.K[5]],
                [msg.K[6], msg.K[7], msg.K[8]]
            ],
            'dist': msg.D
        }
        with open('camera_info.json', 'w') as json_file:
            json.dump(camera_info, json_file, indent=4)

    def aruco_pose_callback(self, msg):
        # Enregistrer la pose reçue dans le CSV
        try:
            timestamp = rospy.Time.now()
            pose = msg.data  # Les données x, y, z sont dans msg.data
            with open(self.csv_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([timestamp.to_sec(), pose[0], pose[1], pose[2]])
                self.pose_counter += 1
            rospy.loginfo(f"Pose enregistrée: x={pose[0]}, y={pose[1]}, z={pose[2]}")
        except Exception as e:
            rospy.logerr(f"Erreur lors de l'enregistrement de la pose: {e}")

    def check_for_timeout(self, event):
        if rospy.Time.now() - self.last_msg_time > rospy.Duration(self.timeout_duration):
            print(f"Nombre d'images sauvegardées: {self.image_counter}")
            rospy.signal_shutdown(f"No messages received for {self.timeout_duration} seconds")

parser = argparse.ArgumentParser(description='Enregistrement des images et des poses du gripper à partir des topics de la caméra')
parser.add_argument('--image_topic', type=str, default='/xtion/rgb/image', help='le topic pour les images de la caméra')
parser.add_argument('--camera_info_topic', type=str, default='/xtion/rgb/camera_info', help='le topic pour les messages d\'info de la caméra')
parser.add_argument('--aruco_pose_topic', type=str, default='/aruco_pose', help='le topic pour les messages de pose ArUco')
parser.add_argument('--image_rate', type=int, default=10, help='sauvegarder une image sur x images')
parser.add_argument('--timeout', type=float, default=2.0, help='Temps d\'attente de message avant arrêt du programme')
parser.add_argument('--gripper_pose_csv', type=str, default='gripper_pose.csv', help='fichier csv ou sauvegarder les poses du gripper')
parser.add_argument('--output_dir', type=str, default='images', help='dossier ou sauvegarder les images')

args = parser.parse_args()

rospy.init_node('topic_recorder', anonymous=True)

recorder = RecordFromTopic(args.image_topic, args.camera_info_topic, args.aruco_pose_topic, args.gripper_pose_csv, args.image_rate, args.timeout, args.output_dir)

rospy.spin()
