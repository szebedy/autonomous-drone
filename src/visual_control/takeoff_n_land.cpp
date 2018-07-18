/**
 * @file offb_node.cpp
 * @brief offboard node, written with mavros version 0.26.0, px4 flight
 * stack 1.8.0 and tested in Gazebo SITL
 */

#include "include/drone_control.h"
#include "include/ros_client.h"

int main(int argc, char **argv)
{
  // The setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(ROS_RATE);

  static tf2_ros::TransformListener tfListener(tfBuffer);

  // Wait for FCU connection
  while(ros::ok() && current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
    ROS_INFO("connecting to FCU...");
  }

  last_svo_estimate = ros::Time::now();

  offboardMode();

  takeOff();

  initVIO();

  //testFlightHorizontal();
  testFlightVertical();

  //turnTowardsMarker();

  //approachMarker();

  land();
  disarm();

  while(ros::ok() && KEEP_ALIVE)
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
