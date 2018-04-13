#include <ros/ros.h>
#include "robot_pose_publisher.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_pose_publisher");
  ros::NodeHandle n("~");

  whycon::RobotPosePublisher robot_pose_publisher(n);
  ros::spin();
}

