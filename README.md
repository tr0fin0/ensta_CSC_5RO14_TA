# ensta_CSC_5RO14_TA
Hardware and Software architecture for robotics.

## Installation

```bash
sudo apt install code
```

After open the desired folder to clone the repository by running:

```bash
git clone https://github.com/tr0fin0/ensta_CSC_5RO14_TA.git CSC_5RO14_TA
```

Then install the projects requirements with:

```bash
chmod +755 installation.sh
./installation.sh
```

File built from instructions presented [here](https://github.com/ailabspace/turtlebot2-noetic/blob/main/install.md). Different verification files may be found in [verification](./docs/verification.md).

## Usage





\




create a catkin package

$ cd ~/catkin_ws/src/
$ catkin_create_pkg autonomous_navigation rospy tf2_ros sensor_msgs nav_msgs





cd ~/catkin_ws && caktin_make
source devel/setup.bash



rosnode list
rosnode info /<node>
rostopic echo /<topic>
top -i





\







Camera

Open Terminal 1
roscore

Open Terminal 2
roslaunch openni2_launch openni2.launch

Open Terminal 3
In the “rqt” window select “Plugins” -> “Visualization” -> “Image View“
rqt

https://gist.github.com/tzutalin/175776fe02a9496a7778


