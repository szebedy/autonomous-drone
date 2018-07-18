/**
 * @file offb_node.cpp
 * @brief offboard node, written with mavros version 0.26.0, px4 flight
 * stack 1.8.0 and tested in Gazebo SITL
 */

#include "include/drone_control.h"

int main(int argc, char **argv)
{
  DroneControl drone_control;
  static tf2_ros::TransformListener tfListener(drone_control.tfBuffer_);

  // The setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(ROS_RATE);

  // Wait for FCU connection
  while(ros::ok() && drone_control.current_state_.connected)
  {
    ros::spinOnce();
    rate.sleep();
    ROS_INFO("connecting to FCU...");
  }

  drone_control.offboardMode();

  drone_control.vioOff();

  drone_control.takeOff();

  //drone_control.initVIO();

  //drone_control.testFlightHorizontal();
  drone_control.testFlightVertical();

  drone_control.hover(10);

  //drone_control.turnTowardsMarker();

  //drone_control.approachMarker();

  drone_control.land();
  drone_control.disarm();

  while(ros::ok() && KEEP_ALIVE)
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
