# Installation
## Install ROS
Open a terminal and then copy and paste the following commands into it. (each block can be copied and pasted at once)
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
```
sudo apt install ros-kinetic-mavros ros-kinetic-mavros-extras
sudo /opt/ros/kinetic/lib/mavros/install_geographiclib_datasets.sh
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
Download source files from Github
```
cd
git clone https://gitlab.ethz.ch/disco-students/fs18/bszebedy.git catkin_ws
```
```
cd ~/catkin_ws
git submodule update --init --recursive
```
Wait until downloading finish correctly, then run
```
cd ~/catkin_ws/bszebedy_px4
make posix_sitl_default gazebo
```
A window will pop up shows a quadcoter. Close the window, go back to the terminal and press Ctrl+C

Build this ROS package by
```
cd ~/catkin_ws
catkin_make
```
run following command so that rosrun can find our new nodes in your ```visual_control``` package

# Usage
## Run roscore, mavros and this ros nodde

You need to open 5 separate terminals in the root of the repository (Ctrl+Shift+T)

Terminal 1:
```
cd ~/catkin_ws/
roscore
```
Terminal 2:
```
cd ~/catkin_ws/bszebedy_px4
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch
```
Terminal 3:
```
cd ~/catkin_ws/
source ./devel/setup.bash
rosrun whycon whycon camera/image_rect_color:=/camera/rgb/image_rect_color camera/camera_info:=/camera/rgb/camera_info _targets:=1 _inner_diameter:=0.09 _outer_diameter:=0.217
```
Terminal 4:
```
rosrun image_view image_view image:=/whycon/image_out
```
Terminal 5:
```
cd ~/catkin_ws/
source ./devel/setup.bash
rosrun visual_control takeoff_n_land
```
