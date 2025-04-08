#!/bin/bash

# Step 1: Update and Upgrade System
echo "Updating and upgrading system..."
sudo apt update && sudo apt upgrade -y

# Step 2: Install ROS Noetic
echo "Installing ROS Noetic..."
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
chmod 755 ./install_ros_noetic.sh
bash ./install_ros_noetic.sh

# Step 3: Install Dependencies
echo "Installing dependencies..."
sudo apt install -y libusb-dev libftdi-dev liborocos-kdl-dev \
  ros-noetic-sophus ros-noetic-joy ros-noetic-depthimage-to-laserscan \
  ros-noetic-move-base ros-noetic-gmapping ros-noetic-navigation \
  ros-noetic-openni-description ros-noetic-openni-launch ros-noetic-openni2-launch 

sudo apt install -y python-is-python3 python3-pip
sudo apt-get install catkin_lint

# Step 4: Clone Repositories
echo "Cloning TurtleBot repositories..."
cd ~/catkin_ws/src

git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_msgs.git
git clone https://github.com/turtlebot/turtlebot_apps.git
git clone https://github.com/turtlebot/turtlebot_simulator.git
git clone https://github.com/turtlebot/turtlebot_interactions.git

git clone --branch noetic https://github.com/AutonomyLab/create_robot.git

git clone https://github.com/yujinrobot/kobuki_core.git
git clone https://github.com/yujinrobot/kobuki_msgs.git
git clone https://github.com/yujinrobot/kobuki.git
git clone https://github.com/yujinrobot/kobuki_desktop.git

git clone https://github.com/ros-drivers/linux_peripheral_interfaces.git
mv linux_peripheral_interfaces/laptop_battery_monitor .
rm -rf linux_peripheral_interfaces

git clone https://github.com/yujinrobot/yujin_ocs.git
git clone https://github.com/yujinrobot/yocs_msgs.git
mv yujin_ocs/yocs_cmd_vel_mux yujin_ocs/yocs_controllers yujin_ocs/yocs_velocity_smoother .
rm -rf yujin_ocs/

git clone --branch release/0.61-noetic https://github.com/stonier/ecl_tools.git
git clone --branch release/0.61-noetic https://github.com/stonier/ecl_lite.git
git clone --branch release/0.62-noetic https://github.com/stonier/ecl_core.git
git clone --branch release/0.60-noetic https://github.com/stonier/ecl_navigation.git 

git clone https://github.com/ros-drivers/freenect_stack.git

# Step 5: Build and Finalize Setup
echo "Building the workspace..."
cd ~/catkin_ws
source ~/.bashrc
rosdep install --from-paths . --ignore-src -r -y
source ~/.bashrc
catkin_make
source ~/.bashrc

# Step 6: Apply Udev Rules
echo "Setting up udev rules..."
rosrun kobuki_ftdi create_udev_rules

echo "Installation complete!"
