#!/usr/bin/env python

import rospkg
import tf2_ros
import rospy
import tf.transformations as tft
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, TransformStamped
from std_msgs.msg import Float32MultiArray  
import time
import sys
import signal
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

def spawn_box():
    rospy.init_node('spawn_box', anonymous=True)
    
    pose_pub = rospy.Publisher('/aruco_pose', Float32MultiArray, queue_size=1)
    rospy.sleep(1)
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    
    try:
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        rospack = rospkg.RosPack()
        sdf_model_path = rospack.get_path('tiago_description') + '/urdf/aruco.sdf'
        
        with open(sdf_model_path, 'r') as model_file:
            model_xml = model_file.read()
        
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        tf_broadcaster = tf2_ros.TransformBroadcaster()
        """
        aruco_pose_list = [
            [0, 0, 2, 0, 0, 0],                        # No translation, no rotation
            [0, 0, 2.5, np.pi/18, np.pi/18, np.pi/18],   # Translation in X and 10° rotation on all axes
            [0, 0, 3, 2*np.pi/18, 2*np.pi/18, 2*np.pi/18], # Translation in X and 20° rotation on all axes
            [0, 0, 2.5, 3*np.pi/18, 3*np.pi/18, 3*np.pi/18], # Translation in -X and 30° rotation on all axes
            [0, 0.5, 2, 4*np.pi/18, 4*np.pi/18, 4*np.pi/18], # Translation in Y and 40° rotation on all axes
            [0, -0.75, 2, 5*np.pi/18, 5*np.pi/18, 5*np.pi/18], # Translation in -Y and 50° rotation on all axes
            [0.5, 0, 2, 6*np.pi/18, 6*np.pi/18, 6*np.pi/18], # Translation in Z and 60° rotation on all axes
            [-1, 0, 2, 7*np.pi/18, 7*np.pi/18, 7*np.pi/18], # Translation in -Z and 70° rotation on all axes
            [0, 1, 3, 8*np.pi/18, 8*np.pi/18, 8*np.pi/18], # Translation in X and Y and 80° rotation on all axes
            [1, -1, 3, 9*np.pi/18, 9*np.pi/18, 9*np.pi/18], # Translation in -X, -Y, Z and 90° rotation on all axes
            [-1, 1.5, 5, 10*np.pi/18, 10*np.pi/18, 10*np.pi/18], # Translation with larger X, Y, and Z, and 100° rotation on all axes
            [1, -1.5, 4, 11*np.pi/18, 11*np.pi/18, 11*np.pi/18], # Translation in mixed directions with 110° rotation
            [-1.5, 0, 6, 12*np.pi/18, 12*np.pi/18, 12*np.pi/18], # Max X and Z translations with 120° rotation
            [1.5, -2, 0.5, 13*np.pi/18, 13*np.pi/18, 13*np.pi/18], # Min X, Y, and max Z translations with 130° rotation
            [-1, 2, 3.5, 14*np.pi/18, 14*np.pi/18, 14*np.pi/18], # Translation in all positive directions with 140° rotation
            [0.5, -2, 5.5, 15*np.pi/18, 15*np.pi/18, 15*np.pi/18], # Mixed translations with 150° rotation
            [1, 1, 1, 16*np.pi/18, 16*np.pi/18, 16*np.pi/18], # Translation of 1 unit in all axes with 160° rotation
            [-1, -1, 2, 17*np.pi/18, 17*np.pi/18, 17*np.pi/18], # Translation in negative Y and Z with 170° rotation
            [1, 0, 4, np.pi, np.pi, np.pi], # Rotation of 180° on all axes
            [-0.5, 1, 0.5, np.pi/3, np.pi/4, np.pi/6], # Complex mixed translation and rotation
            [1.2, -1.5, 5, np.pi/2, np.pi/3, np.pi/4], # Translation with half rotations on each axis
            [-0.8, 0.8, 3, np.pi/5, np.pi/7, np.pi/9], # Diverse rotation angles with mixed translations
            [-1.5, 2, 6, 2*np.pi/3, np.pi/2, np.pi/8], # Max translations with varying rotations
            [1.3, -1.8, 4.5, np.pi/4, np.pi/6, 3*np.pi/8], # Larger translation with fractional rotations
            [1.4, -0.9, 0.6, np.pi/3, np.pi/5, np.pi/7], # Small translations with specific rotation steps
            [-1.1, 1.7, 5.8, 5*np.pi/18, 4*np.pi/18, 3*np.pi/18], # High translation with moderate rotations
            [1.8, -1.3, 2.2, 7*np.pi/18, 6*np.pi/18, 5*np.pi/18] # Varied translation and rotation angles
        ]
        """
        aruco_pose_list = [
            [0, 0, 2, 0, 0, 0],                        # No translation, no rotation
            [0, 0, 2.5, np.pi/18, np.pi/18, np.pi/18],   # Translation in Z and 10° rotation on all axes
            [0, 0, 3, 2*np.pi/18, 2*np.pi/18, 2*np.pi/18], # Translation in Z and 20° rotation on all axes
            [1.5, -1.5, 3.5, 3*np.pi/18, 4*np.pi/18, 5*np.pi/18], # Complex translation with different rotation on each axis
            [-2, 2, 4, 5*np.pi/18, -3*np.pi/18, np.pi/2], # Negative and positive translation with pitch rotation
            [2.5, -1.5, 5, 6*np.pi/18, 8*np.pi/18, np.pi/3], # Translation in different directions and high rotations
            [3, 3, 3, np.pi/4, np.pi/4, np.pi/4], # Translation in all positive directions and equal rotations
            [-2, 3, -3, np.pi/3, -np.pi/3, np.pi/6], # Negative translation with complex rotations
            [1.5, -2.5, 2.5, 3*np.pi/2, np.pi/5, -np.pi/8], # Mixed rotation with higher value translation
            [-3, 2, -2, np.pi, np.pi/4, np.pi/3], # Negative translation in X and Z with complex rotations
            [0, 4, -4, np.pi/6, np.pi/8, np.pi/2], # High translation in Y with diverse rotations
            [3.5, -3.5, 2, -np.pi/4, -np.pi/6, np.pi/7], # Mixed positive and negative translations with mixed rotations
            [5, 0, 0, 2*np.pi/3, -np.pi/3, np.pi/4], # High translation in X with mixed rotations
            [-3.5, 3.5, 3.5, 5*np.pi/18, 7*np.pi/18, -3*np.pi/18], # High translation in all axes with moderate rotations
            [4, -4, -4, 3*np.pi/5, -np.pi/9, np.pi/3], # Maximum translation in negative and positive directions with specific rotations
            [0, 0, -5, np.pi/4, np.pi/4, -np.pi/4], # Maximum negative Z translation with diagonal rotation
            [-5, -5, 5, np.pi, np.pi/6, np.pi/2], # High negative translation in X and Y and rotation on all axes
            [3.3, -1.1, 1.1, np.pi/7, -np.pi/3, np.pi/5], # Mixed translation and rotation on all axes
            [1.1, 5.5, -1.5, 3*np.pi/8, 5*np.pi/6, 2*np.pi/3], # High positive translation in Y with varied rotations
            [4.5, 4.5, 0, -np.pi/4, np.pi/3, -np.pi/6], # Positive translation in X and Y with rotation on roll and yaw
            [-2.2, -3.3, 4.4, np.pi/2, np.pi/4, 3*np.pi/8], # Translation in all directions with moderate rotations
            [0.1, 0.2, 5.2, -np.pi/5, 7*np.pi/18, -np.pi/9], # Very small translations in X and Y with larger Z translation and mixed rotations
            [6, -6, 6, np.pi/3, 2*np.pi/5, -np.pi/4], # Maximum translation in all directions with different rotations
            [4.2, -1.7, 2.8, -np.pi/8, np.pi/6, 5*np.pi/18], # Mixed translations with fractional rotations
            [-3.7, 3.1, -3.2, np.pi/4, np.pi/3, np.pi/5], # Mixed translations with positive rotations on all axes
            [5, -4, 3, -3*np.pi/18, 2*np.pi/7, 4*np.pi/9], # High positive and negative translations with different rotation angles
            [-5.5, 4.4, -2.2, 2*np.pi/3, -np.pi/5, np.pi/6], # High negative X and Z translations with mixed rotations
            [3.8, 3.8, -3.8, np.pi/6, -np.pi/6, np.pi/3], # Positive and negative translation with rotations on all axes
            [4.9, -2.9, 1.9, 5*np.pi/18, np.pi/7, -np.pi/4], # High positive X with mixed translations and rotations
            [-4.7, -1.7, 0.7, np.pi/5, -np.pi/3, np.pi/9], # Negative translation with mixed moderate rotations
            [6.5, 2.5, -1.5, np.pi/8, 2*np.pi/9, 3*np.pi/10], # Very high translation with varied rotations
            [-6, -3, 3, 3*np.pi/5, -2*np.pi/7, 5*np.pi/8], # High negative X and Y with mixed rotations
            [0, 7, -5, -np.pi/4, np.pi/8, -2*np.pi/3], # High Y translation with mixed rotations
            [7.5, -7, 1.5, 5*np.pi/9, np.pi/2, np.pi/4], # Extreme X and Y translations with mixed rotations
            [-7, 7, -7, 3*np.pi/8, -np.pi/6, np.pi/6], # High translation in all axes with moderate rotations
            [4, -4, 8, -np.pi/3, 2*np.pi/7, np.pi/9], # High Z translation with mixed moderate rotations
            [8, 8, -8, np.pi/5, -2*np.pi/3, 3*np.pi/4] # Maximum positive and negative translation with complex rotations
        ]

        initial_pose = [0, 0, 2]
        quaternion_180z = tft.quaternion_from_euler(0, 0, 3.14159) 
        quaternion_90y = tft.quaternion_from_euler(0, 1.5708, 0)   
        quaternion_180x = tft.quaternion_from_euler(3.14159, 0, 0)

        initial_rota = tft.quaternion_multiply(quaternion_90y, quaternion_180x)
        rate_1 = rospy.Rate(1)
        rate_4 = rospy.Rate(0.25)
        
        for idx, pose in enumerate(aruco_pose_list):
            try:
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = "xtion_rgb_optical_frame"  
                transform.child_frame_id = f"aruco_frame"

                transform.transform.translation.x = pose[0]
                transform.transform.translation.y = pose[1]
                transform.transform.translation.z = pose[2]

                quat = tft.quaternion_from_euler(pose[3], pose[4], pose[5])
                transform.transform.rotation.x = quat[0]
                transform.transform.rotation.y = quat[1]
                transform.transform.rotation.z = quat[2]
                transform.transform.rotation.w = quat[3]

                tf_broadcaster.sendTransform(transform)
                print("tf publié")
                rospy.sleep(1.0)
                transform_world = tf_buffer.lookup_transform('map', 'xtion_rgb_optical_frame', rospy.Time(0), rospy.Duration(5.0))

                rospy.loginfo(f"Transformation received between world and {transform.child_frame_id}")

                box_pose = Pose()
                box_pose.position.x = transform_world.transform.translation.x
                box_pose.position.y = transform_world.transform.translation.y
                box_pose.position.z = transform_world.transform.translation.z

                box_pose.orientation.x = transform_world.transform.rotation.x
                box_pose.orientation.y = transform_world.transform.rotation.y
                box_pose.orientation.z = transform_world.transform.rotation.z
                box_pose.orientation.w = transform_world.transform.rotation.w

                model_name = f'aruco_model'
                spawn_model_prox(model_name, model_xml, '', box_pose, 'world')
                rospy.loginfo(f"Aruco créé pour la pose numéro {idx}")

                rate_4.sleep()
                delete_model(model_name)
                rate_1.sleep()

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f"Error fetching transform: {e}")
                return

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
