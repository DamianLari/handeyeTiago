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
    rospy.init_node('broadNode', anonymous=True)
    
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
        tf_broadcaster = tf2_ros.TransformBroadcaster()
        """ 
        aruco_pose_list = [
            [0, 0, 2, 0, 0, 0],                        # No translation, no rotation
            [0, 0, 2.5, np.pi/18, np.pi/18, np.pi/18],  # Translation in Z and small rotation on all axes
            [0, 0, 3, 2*np.pi/18, 2*np.pi/18, 2*np.pi/18], # Translation in Z with moderate rotation
            [1.5, -1.5, 3.5, 3*np.pi/18, 4*np.pi/18, 5*np.pi/18], # Complex translation with different rotation on each axis
            [-2, 2, 4, 5*np.pi/18, -3*np.pi/18, np.pi/2], # Negative and positive translation with pitch rotation
            [2.5, -1.5, 5, 6*np.pi/18, 8*np.pi/18, np.pi/3], # Translation in different directions with larger rotations
            [3, 3, 3, np.pi/4, np.pi/4, np.pi/4], # Equal rotations in all positive directions
            [-2, 3, -3, np.pi/3, -np.pi/3, np.pi/6], # Negative translations with varied rotations
            [1.5, -2.5, 2.5, 3*np.pi/2, np.pi/5, -np.pi/8], # Higher value translation with mixed rotation
            [-3, 2, -2, np.pi, np.pi/4, np.pi/3], # Negative translation with full rotation on roll
            [0, 4, -4, np.pi/6, np.pi/8, np.pi/2], # Translation in Y and negative Z with moderate rotations
            [3.5, -3.5, 2, -np.pi/4, -np.pi/6, np.pi/7], # Mixed positive and negative translations with rotations
            [5, 0, 0, 2*np.pi/3, -np.pi/3, np.pi/4], # High translation in X with moderate rotations
            [-3.5, 3.5, 3.5, 5*np.pi/18, 7*np.pi/18, -3*np.pi/18], # Moderate translations in all axes with varied rotations
            [4, -4, -4, 3*np.pi/5, -np.pi/9, np.pi/3], # Translation with mixed rotations
            [0, 0, -5, np.pi/4, np.pi/4, -np.pi/4], # Maximum negative Z translation with diagonal rotation
            [-5, -5, 5, np.pi, np.pi/6, np.pi/2], # High negative translations with full roll rotation
            [3.3, -1.1, 1.1, np.pi/7, -np.pi/3, np.pi/5], # Mixed translations with rotations
            [1.1, 5.5, -1.5, 3*np.pi/8, 5*np.pi/6, 2*np.pi/3], # High Y translation with varied rotations
            [4.5, 4.5, 0, -np.pi/4, np.pi/3, -np.pi/6], # Mixed translations and rotations on roll and yaw
            [-2.2, -3.3, 4.4, np.pi/2, np.pi/4, 3*np.pi/8], # Moderate rotations on all axes
            [0.1, 0.2, 5.2, -np.pi/5, 7*np.pi/18, -np.pi/9], # Small translations with large Z translation
            [6, -6, 6, np.pi/3, 2*np.pi/5, -np.pi/4], # Maximum translations with different rotations
            [4.2, -1.7, 2.8, -np.pi/8, np.pi/6, 5*np.pi/18], # Mixed translations and fractional rotations
            [-3.7, 3.1, -3.2, np.pi/4, np.pi/3, np.pi/5], # Mixed translations with positive rotations on all axes
            [5, -4, 3, -3*np.pi/18, 2*np.pi/7, 4*np.pi/9], # Mixed translations with different rotation angles
            [-5.5, 4.4, -2.2, 2*np.pi/3, -np.pi/5, np.pi/6], # High negative translations with mixed rotations
            [3.8, 3.8, -3.8, np.pi/6, -np.pi/6, np.pi/3], # Mixed translations and rotations on all axes
            [4.9, -2.9, 1.9, 5*np.pi/18, np.pi/7, -np.pi/4], # Mixed translations and moderate rotations
            [-4.7, -1.7, 0.7, np.pi/5, -np.pi/3, np.pi/9], # Negative translations with mixed rotations
            [6.5, 2.5, -1.5, np.pi/8, 2*np.pi/9, 3*np.pi/10], # Very high translations with varied rotations
            [-6, -3, 3, 3*np.pi/5, -2*np.pi/7, 5*np.pi/8], # High negative translations with rotations
            [0, 7, -5, -np.pi/4, np.pi/8, -2*np.pi/3], # High translation in Y with mixed rotations
            [7.5, -7, 1.5, 5*np.pi/9, np.pi/2, np.pi/4], # Extreme translations with mixed rotations
            [-7, 7, -7, 3*np.pi/8, -np.pi/6, np.pi/6], # High translation in all axes with moderate rotations
            [4, -4, 8, -np.pi/3, 2*np.pi/7, np.pi/9], # High Z translation with mixed moderate rotations
            [8, 8, -8, np.pi/5, -2*np.pi/3, 3*np.pi/4], # Maximum positive and negative translation with complex rotations
            [-8, 9, 9, 4*np.pi/9, np.pi/2, -np.pi/3], # Larger translations with mixed rotations
            [9, -9, -9, -np.pi/5, 3*np.pi/8, np.pi/6], # Very high translation in X, Y, and Z with rotations
            [10, 10, 0, np.pi/2, -np.pi/8, 2*np.pi/3], # Maximum positive translation in X and Y with high rotations
            [-10, -10, 10, 3*np.pi/7, 2*np.pi/5, -np.pi/4], # Maximum translations with varied rotations
            [5, -5, 12, np.pi/6, -np.pi/6, np.pi/3], # High Z translation with mixed moderate rotations
            [11, 11, -11, 4*np.pi/5, np.pi/3, np.pi/8] # Maximum positive translation in all directions with mixed rotations
        ]
        """
        initial_pose = [0, 0, 2]
        quaternion_180z = tft.quaternion_from_euler(0, 0, 3.14159) 
        quaternion_90y = tft.quaternion_from_euler(0, 1.5708, 0)   
        quaternion_180x = tft.quaternion_from_euler(3.14159, 0, 0)

        initial_rota = tft.quaternion_multiply(quaternion_90y, quaternion_180x)
        rate_1 = rospy.Rate(1)
        rate_4 = rospy.Rate(0.25)
        is_aruco = False
        aruco_pose_list = generate_random_poses(100)
        for idx, pose in enumerate(aruco_pose_list):
                try:
                    transform = TransformStamped()
                    transform.header.stamp = rospy.Time.now()
                    transform.header.frame_id = 'xtion_rgb_optical_frame'
                    transform.child_frame_id = 'aruco_frame'
                    model_name = 'aruco_model'

                    transform.transform.translation.x = pose[0]
                    transform.transform.translation.y = pose[1]
                    transform.transform.translation.z = pose[2]

                    quat = tft.quaternion_from_euler(pose[3], pose[4], pose[5])
                    transform.transform.rotation.x = quat[0]
                    transform.transform.rotation.y = quat[1]
                    transform.transform.rotation.z = quat[2]
                    transform.transform.rotation.w = quat[3]

                    tf_broadcaster.sendTransform(transform)
                    print(f"tf numéro {idx} publié")
                    time.sleep(1)

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

def generate_random_poses(num_poses):
    aruco_pose_list = []
    for _ in range(num_poses):
        x = np.random.uniform(-10, 10)  
        y = np.random.uniform(-10, 10) 
        z = np.random.uniform(-10, 12) 

        roll = np.random.uniform(-np.pi, np.pi) 
        pitch = np.random.uniform(-np.pi, np.pi)  
        yaw = np.random.uniform(-np.pi, np.pi) 

        # Ajouter la pose à la liste
        aruco_pose_list.append([x, y, z, roll, pitch, yaw])

    return aruco_pose_list


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
