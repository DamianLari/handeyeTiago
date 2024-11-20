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
        aruco_transla_list = []

        for x in range(-4, 4):
            aruco_transla_list.append([x * 0.5, 0, 0])  

        for y in range(-4, 4):
            aruco_transla_list.append([0, y * 0.3, 0])  

        for z in range(0,21):
            aruco_transla_list.append([0, 0, z * 0.5]) 


        initial_pose = [0, 0, 2]
        quaternion_180z = tft.quaternion_from_euler(0, 0, 3.14159) 
        quaternion_90y = tft.quaternion_from_euler(0, 1.5708, 0)   
        quaternion_180x = tft.quaternion_from_euler(3.14159, 0, 0)

        initial_rota = tft.quaternion_multiply(quaternion_90y, quaternion_180x)
        rate_1 = rospy.Rate(1)
        rate_4 = rospy.Rate(0.25)
        rospy.loginfo("Phase 2 : Rotation uniquement")
        for idx, transla in enumerate(aruco_transla_list):
            try:
                transform: TransformStamped = tf_buffer.lookup_transform('map', 'xtion_rgb_optical_frame', rospy.Time(0), rospy.Duration(5.0))
                rospy.loginfo("Transformation received between world and xtion_rgb_optical_frame")

                box_pose = Pose()

                HTete=np.eye(4)
                HTete[0:3,0:3]= R.from_quat([transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w]).as_matrix()
                HTete[0:3,3]  =[transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z]

                Rot_perpendiculaire=np.eye(3)
                Rot_perpendiculaire= R.from_euler("ZYX",[-math.pi/2,math.pi,0],degrees=False).as_matrix()
                Rot_wanted=R.from_euler("ZYX",[0,0,0],degrees=False).as_matrix()

                Rotation_Aruco=np.eye(4)
                Rotation_Aruco[0:3,0:3]=Rot_wanted.dot(Rot_perpendiculaire)

                Translation_Aruco=np.eye(4)
                Translation_Aruco[0:3, 3] = np.array(initial_pose[:3]) + np.array(transla[:3])

                Rot_wanted_4x4 = np.eye(4)
                Rot_wanted_4x4[:3, :3] = Rot_wanted

                HTag = np.eye(4)
                HTag = Translation_Aruco.dot(Rot_wanted_4x4)
                
                Haruco=np.eye(4)
                #Haruco= Rotation_Aruco.dot(Translation_Aruco)
                Haruco= Translation_Aruco.dot(Rotation_Aruco)
                #arucoCamEulerAngle = np.flip(R.from_matrix(Haruco[0:3,0:3]).as_euler('ZYX')) #avec initial rota
                arucoCamEulerAngle = np.flip(R.from_matrix(HTag[0:3,0:3]).as_euler('ZYX')) # sans initial rota

                #Haruco[0:3,0:3]= Rot_wanted.dot(Rot_perpendiculaire)
                #Haruco[0:3,3]=initial_pose

                Hworld= HTete.dot(Haruco)

                box_pose.position.x = Hworld[0,3]
                box_pose.position.y = Hworld[1,3]
                box_pose.position.z = Hworld[2,3]


                combined_quaternion = R.from_matrix(Hworld[0:3,0:3]).as_quat()
                box_pose.orientation.x = combined_quaternion[0]
                box_pose.orientation.y = combined_quaternion[1]
                box_pose.orientation.z = combined_quaternion[2]
                box_pose.orientation.w = combined_quaternion[3]
                
                #combined_euler = tft.euler_from_quaternion(combined_quaternion)
                #publish_pose(pose_pub, Hworld[0,3],Hworld[1,3],Hworld[2,3],*combined_euler)
                #publish_pose(pose_pub, initial_pose[0],initial_pose[1],initial_pose[2],*arucoCamEulerAngle)
                publish_pose(pose_pub, *(np.array(initial_pose[:3]) + np.array(transla[:3])),*arucoCamEulerAngle)
                rate_1.sleep()

                model_name = f'aruco_model_rotation_{idx}'
                spawn_model_prox(model_name, model_xml, '', box_pose, 'world')
                rospy.loginfo(f"Aruco créé pour la pose de rotation numéro {idx}")
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
