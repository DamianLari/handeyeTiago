#!/usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
import json
import argparse
import shutil
import threading
import rosbag
from geometry_msgs.msg import PoseStamped

class RecordFromTopic:
    def __init__(self, image_topic, camera_info_topic, image_rate, max_images):
        self.image_topic = image_topic
        self.camera_info_topic = camera_info_topic
        self.image_rate = image_rate
        self.max_images = max_images
        self.bridge = CvBridge()

        self.last_msg_time = rospy.Time.now()

        self.image_counter = 0
        self.save_counter = 0
        self.pose_counter = 0

        self.rosbag_name = 'tiagobag.bag'
        
        self.cleanup()

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
        self.nbr_time_gripper_moved = 0
        rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)

        self.monitor_thread = threading.Thread(target=self.monitor_gripper)
        self.monitor_thread.start()

        self.bag = rosbag.Bag(self.rosbag_name, 'w')
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("RecordFromTopic initialized")

    def cleanup(self):
        if os.path.exists(self.rosbag_name):
            os.remove(self.rosbag_name)
        rospy.loginfo("Cleaned up old rosbag file")

    def image_callback(self, msg):
        self.last_msg_time = rospy.Time.now()
        self.save_counter += 1
        if self.save_counter % self.image_rate == 0:
            self.camera_msg = msg
            #rospy.loginfo("Image message received and saved for processing")

    def monitor_gripper(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.image_counter >= self.max_images:
                rospy.loginfo(f"Reached the maximum number of {self.max_images} images. Shutting down.")
                self.shutdown()
                rospy.signal_shutdown("Maximum number of images reached.")
                return

            with self.lock:
                if self.camera_msg and self.is_gripper_immobile() and (rospy.Time.now() - self.last_gripper_movement_time).to_sec() > 1.0:
                    try:
                        self.bag.write(self.image_topic, self.camera_msg)
                        self.image_counter += 1
                        rospy.loginfo(f"Saved image {self.image_counter}/{self.max_images}")
                        self.set_gripper_pose()
                    except CvBridgeError as e:
                        rospy.logerr("CvBridge Error: {0}".format(e))
            rate.sleep()

    def is_gripper_immobile(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.frame_id, self.child_frame_id, rospy.Time(0), rospy.Duration(1.0))
            current_pose = (
                round(trans.transform.translation.x, 3),
                round(trans.transform.translation.y, 3),
                round(trans.transform.translation.z, 3),
                round(trans.transform.rotation.x, 3),
                round(trans.transform.rotation.y, 3),
                round(trans.transform.rotation.z, 3),
                round(trans.transform.rotation.w, 3)
            )
            if self.last_gripper_pose is not None and self.last_gripper_pose == current_pose:
                rospy.loginfo("Gripper is immobile")
                return True
            else:
                self.last_gripper_pose = current_pose
                self.last_gripper_movement_time = rospy.Time.now()
                self.nbr_time_gripper_moved += 1
                #print(f"Gripper moved {self.nbr_time_gripper_moved} times")
                #print(f"Current pose: {current_pose}and last pose: {self.last_gripper_pose}")
                rospy.loginfo("Gripper moved to a new position")
                return False
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transformation not found: {e}")
            return False

    def transformstamped_to_posestampeds(self, transform_stamped):
        pose_stamped = PoseStamped()
        pose_stamped.header = transform_stamped.header
        pose_stamped.pose.position = transform_stamped.transform.translation
        pose_stamped.pose.orientation = transform_stamped.transform.rotation
        return pose_stamped

    def set_gripper_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.frame_id, self.child_frame_id, rospy.Time(0), rospy.Duration(1.0))
            self.last_gripper_pose = self.current_gripper_pose
            self.current_gripper_pose = self.transformstamped_to_posestampeds(trans)
            rospy.loginfo("Recording gripper pose")
            self.bag.write('/gripper/pose', self.current_gripper_pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transformation not found: {e}")

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
        #rospy.loginfo("Camera info message received")

    def shutdown(self):
        if self.bag is not None:
            self.bag.close()
            rospy.loginfo("Rosbag closed")

parser = argparse.ArgumentParser(description='Enregistrement des images et des poses du gripper à partir des topics de la caméra')
parser.add_argument('--image_topic', type=str, default='/xtion/rgb/image', help='le topic pour les images de la caméra')
parser.add_argument('--camera_info_topic', type=str, default='/xtion/rgb/camera_info', help='le topic pour les messages d\'info de la caméra')
parser.add_argument('--image_rate', type=int, default=10, help='sauvegarder une image sur x images')
parser.add_argument('--max_images', type=int, default=300, help='nombre maximum d\'images à enregistrer avant d\'arrêter le programme')
args = parser.parse_args()

rospy.init_node('topic_recorder', anonymous=True)

recorder = RecordFromTopic(args.image_topic, args.camera_info_topic, args.image_rate, args.max_images)

rospy.spin()
