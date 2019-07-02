# Description

This repository intends to enable autonomous drone delivery with the [Intel Aero RTF drone](https://www.intel.com/content/www/us/en/products/drones/aero-ready-to-fly.html) and [PX4](http://px4.io/) autopilot. The code can be executed both on the real drone or simulated on a PC using Gazebo.
Its core is a robot operating system (ROS) node, which communicates with the PX4 autopilot through [mavros](http://wiki.ros.org/mavros). It uses [SVO 2.0](http://rpg.ifi.uzh.ch/svo2.html) for visual odometry, [WhyCon](https://github.com/lrse/whycon) for visual marker localization and [Ewok](https://github.com/VladyslavUsenko/ewok) for trajectoy planning with collision avoidance.

## Paper

Our paper was presented at the 2019 International Conference on Unmanned Aircraft Systems (ICUAS) and can be found here: https://arxiv.org/abs/1809.08022

## Video
A video complementing the research paper available here: https://youtu.be/_pWsEVFLKYg
# Installation
## Prerequisites
The code in this repository was developed and tested on Ubuntu 16.04. It probably will not work on other operating systems or versions.
## Install ROS
Open a terminal and then copy and paste the following commands into it. Make sure you copy and execute each command line one by one to avoid missing an installation step.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
```
```
sudo rosdep init
rosdep update
```
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Download source files from Github
```
cd
git clone https://github.com/szebedy/autonomous-drone.git catkin_ws
```
```
cd ~/catkin_ws
git submodule update --init --recursive
sudo apt install ./ros-kinetic-mavlink_2018.9.17-1_amd64.deb
```
``` 
sudo apt install ros-kinetic-mavros ros-kinetic-mavros-extras
sudo /opt/ros/kinetic/lib/mavros/install_geographiclib_datasets.sh
```
Install additional dependencies for collision avoidance:
```
sudo apt-get install libeigen3-dev libsuitesparse-dev protobuf-compiler libnlopt-dev ros-kinetic-octomap ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-ros ros-kinetic-sophus
```
## Download and run PX4 Gazebo simulator
Prepare tools for building PX4
```
sudo apt-get update
sudo apt-get install python-argparse git-core wget zip python-empy qtcreator cmake build-essential genromfs -y
sudo apt-get install ant protobuf-compiler libeigen3-dev libopencv-dev openjdk-8-jdk openjdk-8-jre clang-3.5 lldb-3.5 python-toml python-numpy -y
sudo apt-get install python-pip -y
sudo -H pip install pandas jinja2
```
If all installations were successful, run
```
cd ~/catkin_ws/px4
make posix_sitl_default gazebo
```
A window will pop up showing a quadcoter. Close the window, go back to the terminal and press Ctrl+C

Build this ROS package by
```
cd ~/catkin_ws
catkin_make
```
Run following command so that rosrun can find our new nodes in your ```offboard_control``` package
```
source ./devel/setup.bash
```
## Download and install SVO 2.0
Download the binaries using the following form: http://rpg.ifi.uzh.ch/svo2download.html

Then, extract and install it based on the provided instructions in install.pdf. For consistence with this readme, please name the overlay workspace `~/svo_ws/` instead of `~/svo_install_overlay_ws/`

Finally, copy the calibration and launch files into the build directory of SVO
```
cp -r ~/catkin_ws/vio_calibration/svo_aero ~/svo_ws/src/rpg_svo_example/svo_ros/
```

# Usage
## Simulation
You have two options for launching the simulation. You can either launch everything in the same window with a launch file, or you can launch each component separately.

### Launch file
You need to open two terminals (Ctrl+Alt+T, Ctrl+Shift+T) and type the following:

Terminal 1:
```
cd ~/svo_ws/
source ./devel/setup.bash
roslaunch svo_ros intel_aero.launch
```
Terminal 2:
```
cd ~/catkin_ws/
source ./devel/setup.bash
cd ~/catkin_ws/px4
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch offboard_control simulation.launch
```

### Run roscore, px4 with mavros, whycon, image_view and offboard_control
You need to open 7 separate terminals in the root of the repository (Ctrl+Alt+T, 6 x Ctrl+Shift+T)


Terminal 1:
```
cd ~/catkin_ws/px4
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch
```
Terminal 2:
```
cd ~/catkin_ws/
source ./devel/setup.bash
rosrun whycon whycon camera/image_rect_color:=/camera/rgb/image_raw camera/camera_info:=/camera/rgb/camera_info _targets:=1 _inner_diameter:=0.08 _outer_diameter:=0.1952
```
Terminal 3:
```
rosrun image_view image_view image:=/whycon/image_out
```
Terminal 4:
```
cd ~/svo_ws/
source ./devel/setup.bash
roslaunch svo_ros intel_aero.launch
```
Terminal 5:
```
cd ~/catkin_ws/
source ./devel/setup.bash
rosrun offboard_control offboard_control
```
Terminal 6:
```
cd ~/catkin_ws/
source ./devel/setup.bash
rosrun trajectory_planner trajectory_planner
```
Terminal 7:
```
cd ~/catkin_ws/
rviz -d src/trajectory_planner/rviz/simulation.rviz
```
## Intel Aero RTF drone
You have two options for launching the code on the Intel Aero RTF drone. You can either launch everything in the same window with a launch file, or you can launch each component separately.

### Launch file
You need to open two terminals (Ctrl+Alt+T, Ctrl+Shift+T) and type the following:


Terminal 1:
```
cd ~/svo_ws/
source ./devel/setup.bash
roslaunch svo_ros intel_aero.launch
```
Terminal 2:
```
cd ~/catkin_ws/
source ./devel/setup.bash
roslaunch offboard_control intel_aero.launch
```

### Run roscore, mavros, whycon and offboard_control
You need to open 7 separate terminals in the root of the repository (Ctrl+Alt+T, 6 x Ctrl+Shift+T)


Terminal 1:
```
cd ~/catkin_ws/
roslaunch mavros px4.launch fcu_url:=tcp://127.0.0.1:5760
```
Terminal 2:
```
cd ~/catkin_ws/
source ./devel/setup.bash
rosrun whycon whycon camera/image_rect_color:=/camera/rgb/image_rect_color camera/camera_info:=/camera/rgb/camera_info _targets:=1
```
Terminal 3:
```
roslaunch realsense_camera r200_nodelet_rgbd.launch
```
Terminal 4:
```
cd ~/catkin_ws/
source ./devel/setup.bash
roslaunch usb_cam usb_cam.launch
```
Terminal 5:
```
cd ~/svo_ws/
source ./devel/setup.bash
roslaunch svo_ros intel_aero.launch
```
Terminal 6:
```
cd ~/catkin_ws/
source ./devel/setup.bash
rosrun trajectory_planner trajectory_planner
```
Terminal 7:
```
cd ~/catkin_ws/
source ./devel/setup.bash
rosrun offboard_control offboard_control
```
