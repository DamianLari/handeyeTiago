#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
import tf2_ros
import threading

class RobotGripMonitor:
    def __init__(self):
        rospy.init_node('robot_grip_monitor', anonymous=True)

        self.camera_topic = '/xtion/rgb/image'
        self.tf_topic = '/tf'
        
        self.frame_id = "base_link"
        self.child_frame_id = "gripper_right_left_finger_link"

        self.camera_pub = rospy.Publisher('/custom_camera_topic', Image, queue_size=10)
        self.tf_pub = rospy.Publisher('/custom_tf_topic', TFMessage, queue_size=10)

        rospy.Subscriber(self.camera_topic, Image, self.camera_callback)
        rospy.Subscriber(self.tf_topic, TFMessage, self.tf_callback)

        self.lock = threading.Lock()
        self.last_gripper_movement_time = rospy.Time.now()
        self.last_gripper_pose = None
        self.current_gripper_pose = None
        self.camera_msg = None
        self.tf_msgs = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.monitor_thread = threading.Thread(target=self.monitor_gripper)
        self.monitor_thread.start()

    def camera_callback(self, msg):
        with self.lock:
            self.camera_msg = msg
            print("une image de reçue")

    def tf_callback(self, msg):
        with self.lock:
            self.tf_msgs.append(msg)
            print("un tf de reçu")
            self.set_gripper_pose()

    def set_gripper_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.frame_id, self.child_frame_id, rospy.Time(0), rospy.Duration(1.0))
            self.last_gripper_pose = self.current_gripper_pose
            self.current_gripper_pose = (
                round(trans.transform.translation.x, 4),
                round(trans.transform.translation.y, 4),
                round(trans.transform.translation.z, 4),
                round(trans.transform.rotation.x, 4),
                round(trans.transform.rotation.y, 4),
                round(trans.transform.rotation.z, 4),
                round(trans.transform.rotation.w, 4)
            )
            if self.last_gripper_pose is None or self.current_gripper_pose != self.last_gripper_pose:
                self.last_gripper_movement_time = rospy.Time.now()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transformation non trouvée : {e}")

    def monitor_gripper(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.lock:
                if rospy.Time.now() - self.last_gripper_movement_time > rospy.Duration(1.0):
                    if self.camera_msg:
                        self.camera_pub.publish(self.camera_msg)
                    for tf_msg in self.tf_msgs:
                        self.tf_pub.publish(tf_msg)
                    self.tf_msgs = [] 
            rate.sleep()

if __name__ == '__main__':
    try:
        RobotGripMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
