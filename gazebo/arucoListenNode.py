#!/usr/bin/env python

import rospkg
import tf2_ros
import rospy
import tf.transformations as tft
from gazebo_msgs.srv import SpawnModel, DeleteModel,SetModelState
from geometry_msgs.msg import Pose, TransformStamped
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Float32MultiArray  
import time
import sys
import signal
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

def spawn_box():
    rospy.init_node('listenNode', anonymous=True)
    
    pose_pub = rospy.Publisher('/aruco_pose', Float32MultiArray, queue_size=1)
    rospy.sleep(1)
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    rospy.wait_for_service('/gazebo/set_model_state')
    
    try:
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        set_model_state_prox = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        rospack = rospkg.RosPack()
        sdf_model_path = rospack.get_path('tiago_description') + '/urdf/aruco.sdf'
        
        with open(sdf_model_path, 'r') as model_file:
            model_xml = model_file.read()
        
        tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        tf_broadcaster = tf2_ros.TransformBroadcaster()
        

        initial_pose = [0, 0, 2]
        quaternion_180z = tft.quaternion_from_euler(0, 0, 3.14159) 
        quaternion_90y = tft.quaternion_from_euler(0, 1.5708, 0)   
        quaternion_180x = tft.quaternion_from_euler(3.14159, 0, 0)

        initial_rota = tft.quaternion_multiply(quaternion_90y, quaternion_180x)
        rate_1 = rospy.Rate(1)
        rate_4 = rospy.Rate(0.25)
        is_aruco = False
        model_name = 'aruco_model'
        while True:
            try:
                transform_world = tf_buffer.lookup_transform('map', 'xtion_rgb_optical_frame', rospy.Time(0), rospy.Duration(10.0))
                rospy.loginfo(f"Transformation received between world and aruco_frame")

                if is_aruco:
                    box_pose = ModelState()
                    box_pose.model_name = model_name
                    box_pose.pose.position.x = transform_world.transform.translation.x
                    box_pose.pose.position.y = transform_world.transform.translation.y
                    box_pose.pose.position.z = transform_world.transform.translation.z

                    box_pose.pose.orientation.x = transform_world.transform.rotation.x
                    box_pose.pose.orientation.y = transform_world.transform.rotation.y
                    box_pose.pose.orientation.z = transform_world.transform.rotation.z
                    box_pose.pose.orientation.w = transform_world.transform.rotation.w

                    set_model_state_prox(box_pose)
                else:
                    box_pose = Pose()
                    box_pose.position.x = transform_world.transform.translation.x
                    box_pose.position.y = transform_world.transform.translation.y
                    box_pose.position.z = transform_world.transform.translation.z

                    box_pose.orientation.x = transform_world.transform.rotation.x
                    box_pose.orientation.y = transform_world.transform.rotation.y
                    box_pose.orientation.z = transform_world.transform.rotation.z
                    box_pose.orientation.w = transform_world.transform.rotation.w

                    spawn_model_prox(model_name, model_xml, '', box_pose, 'world')
                    rospy.loginfo(f"Aruco créé")
                    rate_4.sleep()
                    is_aruco = True

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f"Error fetching transform: {e}")
                return
        delete_model(model_name)
        rate_1.sleep()
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def delete_model(model_name="aruco_model"):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model_prox(model_name)
        rospy.loginfo(f"Model {model_name} supprimé de la scène")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def signal_handler(sig, frame):
    delete_model()
    sys.exit(0)

def publish_pose(pose_pub, x, y, z, roll, pitch, yaw):
    pose_msg = Float32MultiArray()
    pose_msg.data = [x, y, z, roll, pitch, yaw]
    pose_pub.publish(pose_msg)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    try:
        spawn_box()
    except rospy.ROSInterruptException:
        pass
