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
        
        # Liste de translations
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

        # Liste de rotations
        aruco_rota_list = [
            [0, 0, 0],          # No rotation
            [np.pi/6, 0, 0],    # Roll 30°
            [np.pi/4, 0, 0],    # Roll 45°
            [np.pi/2, 0, 0],    # Roll 90°
            [np.pi, 0, 0],      # Roll 180°
            [-np.pi/2, 0, 0],   # Roll -90°
            [0, np.pi/6, 0],    # Pitch 30°
            [0, np.pi/4, 0],    # Pitch 45°
            [0, np.pi/2, 0],    # Pitch 90°
            [0, np.pi, 0],      # Pitch 180°
            [0, -np.pi/2, 0],   # Pitch -90°
            [0, 0, np.pi/6],    # Yaw 30°
            [0, 0, np.pi/4],    # Yaw 45°
            [0, 0, np.pi/2],    # Yaw 90°
            [0, 0, np.pi],      # Yaw 180°
            [0, 0, -np.pi/2],   # Yaw -90°
        ]

        # Position initiale fixe pour toutes les étapes
        initial_pose = [2, 0, 0]
        quaternion_180z = tft.quaternion_from_euler(0, 0, 3.14159) 
        quaternion_90y = tft.quaternion_from_euler(0, 1.5708, 0)   
        quaternion_180x = tft.quaternion_from_euler(3.14159, 0, 0)

        initial_rota = tft.quaternion_multiply(quaternion_90y, quaternion_180x)
        """

        # PHASE 1 : Appliquer uniquement les translations (sans rotations)
        rospy.loginfo("Phase 1 : Translation uniquement")
        for idx, pose in enumerate(aruco_pose_list):
            try:
                transform: TransformStamped = tf_buffer.lookup_transform('map', 'xtion_rgb_optical_frame', rospy.Time(0), rospy.Duration(5.0))
                rospy.loginfo("Transformation received between world and xtion_rgb_optical_frame")

                box_pose = Pose()

                # Ajouter la translation à l'initial_pose
                box_pose.position.x = transform.transform.translation.x + initial_pose[0] + pose[0]
                box_pose.position.y = transform.transform.translation.y + initial_pose[1] + pose[1]
                box_pose.position.z = transform.transform.translation.z + initial_pose[2] + pose[2]

                # Pas de rotation pour cette phase
                box_pose.orientation.x = initial_rota[0] 
                box_pose.orientation.y = initial_rota[1]
                box_pose.orientation.z = initial_rota[2]
                box_pose.orientation.w = initial_rota[3]

                model_name = f'aruco_model_translation_{idx}'
                
                publish_pose(pose_pub, pose[0], pose[1], pose[2])
                time.sleep(2)
                spawn_model_prox(model_name, model_xml, '', box_pose, 'world')

                rospy.loginfo(f"Aruco créé pour la pose de translation numéro {idx}")
                time.sleep(3)

                # Supprimer le modèle après un certain temps
                delete_model(model_name)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f"Error fetching transform: {e}")
                return
        """
        # PHASE 2 : Appliquer uniquement les rotations (en gardant initial_pose fixe)
        rospy.loginfo("Phase 2 : Rotation uniquement")
        for idx, rota in enumerate(aruco_rota_list):
            try:
                transform: TransformStamped = tf_buffer.lookup_transform('map', 'xtion_rgb_optical_frame', rospy.Time(0), rospy.Duration(5.0))
                rospy.loginfo("Transformation received between world and xtion_rgb_optical_frame")

                box_pose = Pose()

                # Garder la position fixe
                box_pose.position.x = transform.transform.translation.x + initial_pose[0]
                box_pose.position.y = transform.transform.translation.y + initial_pose[1]
                box_pose.position.z = transform.transform.translation.z + initial_pose[2]

                # Appliquer la rotation de la liste aruco_rota_list
                roll_offset = rota[0]
                pitch_offset = rota[1]
                yaw_offset = rota[2]

                # Convertir les angles Euler en quaternion
                quaternion = tft.quaternion_from_euler(roll_offset, pitch_offset, yaw_offset)
                combined_quaternion = tft.quaternion_multiply(quaternion, initial_rota)
                box_pose.orientation.x = combined_quaternion[0]
                box_pose.orientation.y = combined_quaternion[1]
                box_pose.orientation.z = combined_quaternion[2]
                box_pose.orientation.w = combined_quaternion[3]

                model_name = f'aruco_model_rotation_{idx}'
                
                publish_pose(pose_pub, 3.14159, 1.5708, 0)
                time.sleep(2)
                spawn_model_prox(model_name, model_xml, '', box_pose, 'world')

                rospy.loginfo(f"Aruco créé pour la pose de rotation numéro {idx}")
                time.sleep(3)

                # Supprimer le modèle après un certain temps
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
