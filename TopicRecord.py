#!/usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import csv
import json
import argparse
import shutil
import threading

class RecordFromTopic:
    def __init__(self, image_topic, camera_info_topic, outputCSV, image_rate, timeout_duration):
        self.image_topic = image_topic
        self.camera_info_topic = camera_info_topic
        self.image_rate = image_rate
        self.timeout_duration = timeout_duration
        self.bridge = CvBridge()

        self.last_msg_time = rospy.Time.now()

        self.image_counter = 0
        self.save_counter = 0
        self.pose_counter = 0

        self.csv_filename = outputCSV
        
        self.cleanup()

        if not os.path.isfile(self.csv_filename):
            with open(self.csv_filename, 'w', newline='') as csvfile:
                fieldnames = ['time', 'tx', 'ty', 'tz', 'rx', 'ry', 'rz']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.frame_id = "base_link" 
        self.child_frame_id = "gripper_right_left_finger_link" 

        self.lock = threading.Lock()
        self.last_gripper_movement_time = rospy.Time.now()
        self.last_gripper_pose = None
        self.current_gripper_pose = None
        self.camera_msg = None
        self.tf_msgs = []

        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)

        self.monitor_thread = threading.Thread(target=self.monitor_gripper)
        self.monitor_thread.start()

        rospy.Timer(rospy.Duration(1), self.check_for_timeout)

    def cleanup(self):
        if os.path.exists('images'):
            shutil.rmtree('images')
        os.makedirs('images')

        if os.path.exists(self.csv_filename):
            os.remove(self.csv_filename)

    def image_callback(self, msg):
        self.last_msg_time = rospy.Time.now()
        self.save_counter += 1 
        if self.save_counter % self.image_rate == 0:  
            self.camera_msg = msg

    def monitor_gripper(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.lock:
                if self.camera_msg and (rospy.Time.now() - self.last_gripper_movement_time > rospy.Duration(1.0)):
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(self.camera_msg, "bgr8")
                        filename = f'images/image_{self.image_counter:04d}.jpg'
                        cv2.imwrite(filename, cv_image)
                        self.image_counter += 1
                        self.set_gripper_pose()
                    except CvBridgeError as e:
                        rospy.logerr("CvBridge Error: {0}".format(e))
            rate.sleep()

    def set_gripper_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.frame_id, self.child_frame_id, rospy.Time(0), rospy.Duration(1.0))
            self.last_gripper_pose = self.current_gripper_pose
            with open(self.csv_filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                self.current_gripper_pose = (
                    round(trans.transform.translation.x, 4),
                    round(trans.transform.translation.y, 4),
                    round(trans.transform.translation.z, 4),
                    round(trans.transform.rotation.x, 4),
                    round(trans.transform.rotation.y, 4),
                    round(trans.transform.rotation.z, 4),
                    round(trans.transform.rotation.w, 4)
                )
                writer.writerow([rospy.Time.now(), *self.current_gripper_pose])
                if self.last_gripper_pose is None or self.current_gripper_pose != self.last_gripper_pose:
                    self.pose_counter += 1
                    self.last_gripper_movement_time = rospy.Time.now()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transformation non trouvée : {e}")

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

    def check_for_timeout(self, event):
        if rospy.Time.now() - self.last_msg_time > rospy.Duration(self.timeout_duration):
            print(f"Nombre d'images sauvegardées : {self.image_counter}")
            print(f"Nombre de poses sauvegardées : {self.pose_counter}")
            rospy.signal_shutdown(f"No messages received for {self.timeout_duration} seconds")


parser = argparse.ArgumentParser(description='Enregistrement des images et des poses du gripper à partir des topics de la caméra')
parser.add_argument('--image_topic', type=str, default='/xtion/rgb/image', help='le topic pour les images de la caméra')
parser.add_argument('--camera_info_topic', type=str, default='/xtion/rgb/camera_info', help='le topic pour les messages d\'info de la caméra')
parser.add_argument('--image_rate', type=int, default=10, help='sauvegarder une image sur x images')
parser.add_argument('--timeout', type=float, default=2.0, help='Temps d\'attente de message avant arrêt du programme')
parser.add_argument('--gripper_pose_csv', type=str, default='gripper_pose.csv', help='fichier csv ou sauvegarder les poses du gripper')
args = parser.parse_args()

rospy.init_node('topic_recorder', anonymous=True)

recorder = RecordFromTopic(args.image_topic, args.camera_info_topic, args.gripper_pose_csv, args.image_rate, args.timeout)

rospy.spin()
