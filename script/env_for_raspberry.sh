### INSTALL ros and related packages
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop -y

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update

sudo apt install -y ros-noetic-tf2-sensor-msgs ros-noetic-move-base-msgs ros-noetic-pcl-conversions ros-noetic-pcl-ros ros-noetic-libg2o

### INSTALL some tools
sudo apt install -y git vim net-tools terminator libgoogle-glog-dev libsdl-image1.2-dev libsdl-image1.2-dev xserver-xorg-video-dummy xserver-xorg-core-hwe-18.04 libbullet-dev

sudo gedit /etc/netplan/01-network-manager-all.yaml
:<<BLOCK'
network:
    renderer: NetworkManager
    ethernets:
        enp1s0:
            dhcp4: false
            addresses:
              - 192.168.1.100/24
            gateway4: 192.168.1.100
            # optional: true
    version: 2
'BLOCK
sudo netplan apply



