ls
cd root
ls
ssh pal@10.191.76.59 
ssh pal@10.191.76.59 
exit
ls
cd root/
ls
source /opt/ros/noetic/setup.sh 
nano /etc/resolv.conf 
source /opt/ros/noetic/setup.sh 
export ROS_MASTER_URI=http://tiago-189c:11311
export ROS_IP=10.191.76.115
ifconfig
apt-get install net-tool
apt-get install net-tools
ifconfig
rostopic list
exit
cat /etc/host
cat /etc/hosts
cat /etc/hosts
ls
cd root
ls
cat run_all.sh 
cat /etc/hosts
source /opt/ros/noetic/setup.sh 
nano /etc/hosts
ping tiago-189c
apt-get install nttools
apt-get install nettools
apt-get update
apt-get install -y iputils-ping
ping tiago-189c
nano /resolv.conf
nano /etc/resolv.conf
ping tiago-189c
ping 10.191.76.59
cat /etc/host
cat /etc/hosts
export ROS_MASTER_URI=http://tiago-189c:11311
export ROS_IP=10.191.76.112
rostopic list
export ROS_MASTER_URI=http://10.191.76.59:11311 
rostopic list
rostopic hz /xtion/rgb/image
source /opt/ros/noetic/setup.sh 
roscore
roscore
ping 10.191.76.112
python3 RecordFromTopic.py --image_topic /xtion/rgb/image --camera_info_topic /xtion/rgb/camera_info --gripper_pose_csv gripper_pose.csv --image_rate 5 --timeout 2.0 --bag_file tiago_rosbag_left.bag
pip install rospy
apt-get update
apt-get install -y python3-pip
apt-get update
apt-get install -y ros-<ros_distro>-rospy
apt-get install -y ros-noetic-rospy
source /opt/ros/<ros_distro>/setup.bash
source /opt/ros/noetic/setup.bash
python3 RecordFromTopic.py --image_topic /xtion/rgb/image --camera_info_topic /xtion/rgb/camera_info --gripper_pose_csv gripper_pose.csv --image_rate 5 --timeout 2.0 --bag_file tiago_rosbag_left.bag
pip install tf2_ros
apt-get install tf2_ros
apt-get install -y ros-noetic-tf2-ros
python3 RecordFromTopic.py --image_topic /xtion/rgb/image --camera_info_topic /xtion/rgb/camera_info --gripper_pose_csv gripper_pose.csv --image_rate 5 --timeout 2.0 --bag_file tiago_rosbag_left.bag
pip install cv2
pip install opencv-python
pip install opencv-contrib-python
python3 RecordFromTopic.py --image_topic /xtion/rgb/image --camera_info_topic /xtion/rgb/camera_info --gripper_pose_csv gripper_pose.csv --image_rate 5 --timeout 2.0 --bag_file tiago_rosbag_left.bag
apt-get install -y ros-noetic-cv-bridge
python3 RecordFromTopic.py --image_topic /xtion/rgb/image --camera_info_topic /xtion/rgb/camera_info --gripper_pose_csv gripper_pose.csv --image_rate 5 --timeout 2.0 --bag_file tiago_rosbag_left.bag
python3 RecordFromTopic.py --image_topic /xtion/rgb/image --camera_info_topic /xtion/rgb/camera_info --gripper_pose_csv gripper_pose.csv --image_rate 5 --timeout 2.0 --bag_file tiago_rosbag_left.bag
source /opt/ros/noetic/setup.sh 
rostopic hz /tf
rosrun
rviz
rviz
rostopic echo /tf_static
rostopic echo /tf_static |grep frames_id
rostopic echo /tf_static |grep frame_id
rostopic echo /tf_static |grep head
rostopic echo /tf_static |grep xtion
rviz
