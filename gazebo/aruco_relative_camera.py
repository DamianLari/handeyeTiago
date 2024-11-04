#!/usr/bin/env python

import rospkg
import tf2_ros
import rospy
#import tf2_geometry_msgs
import tf.transformations as tft
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, TransformStamped
import time
import sys
import signal

def spawn_box():
    rospy.init_node('spawn_box', anonymous=True)
    
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    
    try:
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        rospack = rospkg.RosPack()
        sdf_model_path = rospack.get_path('tiago_description') + '/urdf/aruco.sdf'
        
        with open(sdf_model_path, 'r') as model_file:
            model_xml = model_file.read()
        
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        aruco_pose_list = [[1, 0, 0],
                            [1, 0.5, 0],
                            [1, -0.5, 0],
                            [1.5, 0.25, 0],  
                            [1.5, -0.25, 0],   
                            [2, 0, 0],
                            [2, 0.75, 0],
                            [2, -0.75, 0],
                            [2.5, 0.5, 0],     
                            [2.5, -0.5, 0],    
                            [3, 0, 0],
                            [3, 1, 0],
                            [3, -1, 0],
                            [3.5, 0.75, 0],    
                            [3.5, -0.75, 0],   
                            [4, 0, 0],
                            [4, 1, 0],
                            [4, -1, 0],
                            [4.5, 0.5, 0],    
                            [4.5, -0.5, 0],   
                            [4, 0, 1],         
                            [4, 1, 0],         
                            [4, -1, 0],        
                            [4, 0, 0],        
                            [4, 2, 0],
                            [4, -2, 0],
                            [5, 0, 0],         
                            [5, 0.5, 0],      
                            [5, -0.5, 0],     
                            [5.5, 0, 0],      
                            [5.5, 0.75, 0],    
                            [5.5, -0.75, 0],  
                            [6, 0, 0],         
                            [6, 1, 0],         
                            [6, -1, 0],
                            [1.5, 0.5, 0.5],
                            [2.5, -0.5, 0.5],
                            [3, 0.5, 0.75],
                            [3.5, -0.75, 0.5],
                            [4, 0.5, 1],
                            [4, -1, 1],
                            [5, 1, 1],
                            [5, -0.5, 1],
                            [6, 0.5, 1],
                            [6, -1, 1]
                        ]
        for idx,pose in enumerate(aruco_pose_list):
            try:
                transform: TransformStamped = tf_buffer.lookup_transform('map', 'xtion_rgb_optical_frame', rospy.Time(0), rospy.Duration(5.0))
                rospy.loginfo("Transformation received between world and xtion_rgb_optical_frame")
            
                box_pose = Pose()

                box_pose.position.x = transform.transform.translation.x + pose[0]
                box_pose.position.y = transform.transform.translation.y + pose[1]
                box_pose.position.z = transform.transform.translation.z + pose[2]

                quaternion_180y = tft.quaternion_from_euler(0, 3.14159, 0) 
                quaternion_0 = tft.quaternion_from_euler(0, 0, 0) 
                quaternion_180z = tft.quaternion_from_euler(0, 0, 3.14159) 
                quaternion_90x = tft.quaternion_from_euler(1.5708, 0, 0)   
                quaternion_90y = tft.quaternion_from_euler(0, 1.5708, 0)   
                quaternion_180x = tft.quaternion_from_euler(3.14159, 0, 0)   

                quaternion_combined = tft.quaternion_multiply(quaternion_90y, quaternion_180x)
                box_pose.orientation.x = quaternion_combined[0]
                box_pose.orientation.y = quaternion_combined[1]
                box_pose.orientation.z = quaternion_combined[2]
                box_pose.orientation.w = quaternion_combined[3]
                """
                box_pose.orientation.x = transform.transform.rotation.x
                box_pose.orientation.y = transform.transform.rotation.y
                box_pose.orientation.z = transform.transform.rotation.z
                box_pose.orientation.w = transform.transform.rotation.w
                """

                """
                box_pose.position.x = 0
                box_pose.position.y = 0.0  
                box_pose.position.z = 1

                box_pose.orientation.x = 0.0
                box_pose.orientation.y = 0.0
                box_pose.orientation.z = 0.0
                box_pose.orientation.w = 1.0
                """
                #model_name = 'aruco_model'
                model_name = f'aruco_model_{idx}'
                

                spawn_model_prox(model_name, model_xml, '', box_pose, 'world')

                rospy.loginfo(f"aruco crée pour la pose numéro {idx}")
                time.sleep(3)
                delete_model(model_name)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr("Error fetching transform: %s" % e)
                return
            
            
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def delete_model(model_name="aruco_model"):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model_prox(model_name) 
        rospy.loginfo(f"model {model_name} supprimé de la scne")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def signal_handler(sig, frame):
    delete_model()
    sys.exit(0)

    
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    try:
        spawn_box()
    except rospy.ROSInterruptException:
        pass
