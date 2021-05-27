# this repo is to store the local parameters and the launch files.

## for mapping with a rosbag
roslaunch offline_mapping_3d.launch bag_filenames:="path_to_bag.bag"

rosbag info:
topics:      /imu/data          158137 msgs    : sensor_msgs/Imu        
             /velodyne_points     3921 msgs    : sensor_msgs/PointCloud2



for raspberry 4b+
sudo apt install net-tools
sudo apt install htop -y
sudo apt install ros-noetic-ros-base
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install lm-sensors
sudo apt install exfat-utils 
sudo apt install libpcl-dev 
sudo apt install ros-noetic-pcl-conversions ros-noetic-pcl-msgs ros-noetic-pcl-ros
sudo apt install ros-noetic-urdf
sudo apt install ros-noetic-rviz
sudo apt install ros-noetic-eigen-conversions ros-noetic-tf-conversions
sudo apt install terminator
sudo apt install libdiagnostics-dev 
sudo apt install ros-noetic-diagnostics 
sudo apt install libbullet-dev
sudo apt install libsdl-dev
sudo apt install libsdl-image1.2-dev 
sudo apt install ros-noetic-tf2-sensor-msgs 
sudo apt install ros-noetic-move-base-msgs 
