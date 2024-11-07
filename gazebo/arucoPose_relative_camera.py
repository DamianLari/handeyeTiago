#!/usr/bin/env python

import rospkg
import tf2_ros
import rospy
from scipy.spatial.transform import Rotation as R
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, TransformStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
import sys
import signal

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
        
        aruco_pose_list = [
            [0, 0, 0],
            [0.5, 0, 0], 
            [1, 0, 0],
            [-0.5, 0, 0],
            [-1, 0, 0],
            [0, 0.5, 0],
            [0, 0.75, 0],
            [0, -0.5, 0],
            [0, -0.75, 0],
            [0, 0, 0.5],
            [0, 0, 1],
            [0, 0, -0.5],
            [0, 0, -1]
        ]

        aruco_rota_list = [
            [0, 0, 0],
            [np.pi/6, 0, 0],
            [np.pi/4, 0, 0],
            [np.pi/2, 0, 0],
            [np.pi, 0, 0],
            [-np.pi/2, 0, 0],
            [0, np.pi/6, 0],
            [0, np.pi/4, 0],
            [0, np.pi/2, 0],
            [0, np.pi, 0],
            [0, -np.pi/2, 0],
            [0, 0, np.pi/6],
            [0, 0, np.pi/4],
            [0, 0, np.pi/2],
            [0, 0, np.pi],
            [0, 0, -np.pi/2],
        ]

        initial_translation = np.array([2, 0, 0])
        initial_rotation = R.from_euler('ZYX', [3.14159, 1.5708, 0])

        for idx, (translation, rotation) in enumerate(zip(aruco_pose_list, aruco_rota_list)):
            try:
                transform: TransformStamped = tf_buffer.lookup_transform('map', 'xtion_rgb_optical_frame', rospy.Time(0), rospy.Duration(5.0))
                rospy.loginfo("Transformation received between world and xtion_rgb_optical_frame")
                
                H_camera = np.eye(4)
                camera_rotation = R.from_quat([transform.transform.rotation.x,
                                               transform.transform.rotation.y,
                                               transform.transform.rotation.z,
                                               transform.transform.rotation.w])
                H_camera[0:3, 0:3] = camera_rotation.as_matrix()
                H_camera[0:3, 3] = [transform.transform.translation.x,
                                    transform.transform.translation.y,
                                    transform.transform.translation.z]
                
                #H_aruco = np.eye(4)
                #H_aruco[0:3, 3] = initial_translation + translation  

                #aruco_rotation = R.from_euler('zyx', rotation)
                aruco_rotation = R.from_euler('ZYX', np.flip(rotation))
                initial_rot_matrix = initial_rotation * aruco_rotation  
                #H_aruco[0:3, 0:3] = initial_rot_matrix.as_matrix()

                H_Trans = np.eye(4)
                H_Trans[0:3, 3] = initial_translation + translation 
                H_Rot= initial_rot_matrix.as_matrix()

                H_aruco= H_Rot.dot(H_Trans)


                H_world = np.dot(H_camera, H_aruco)
                
                box_pose = Pose()
                box_pose.position.x = H_world[0, 3]
                box_pose.position.y = H_world[1, 3]
                box_pose.position.z = H_world[2, 3]

                world_rotation = R.from_matrix(H_world[0:3, 0:3])
                quaternion = world_rotation.as_quat()
                box_pose.orientation.x = quaternion[0]
                box_pose.orientation.y = quaternion[1]
                box_pose.orientation.z = quaternion[2]
                box_pose.orientation.w = quaternion[3]
                
                model_name = f'aruco_model_{idx}'
                
                publish_pose(pose_pub, translation[0], translation[1], translation[2])
                time.sleep(1)
                spawn_model_prox(model_name, model_xml, '', box_pose, 'world')

                rospy.loginfo(f"Aruco créé pour la pose numéro {idx}")
                time.sleep(3)
                
                delete_model(model_name)
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

def publish_pose(pose_pub, x, y, z):
    pose_msg = Float32MultiArray()
    pose_msg.data = [x, y, z]
    pose_pub.publish(pose_msg)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    try:
        spawn_box()
    except rospy.ROSInterruptException:
        pass
