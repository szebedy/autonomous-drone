#include "include/ros_client.h"
#include "include/drone_control.h"

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

ROSClient::ROSClient() {}

void ROSClient::init(DroneControl *drone_control)
{
  int argc = 0;
  ros::init(argc, 0, "offb_node");

  ros::NodeHandle nh;

  state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &DroneControl::state_cb, drone_control);
  marker_pos_sub_ = nh.subscribe<geometry_msgs::PoseArray>("/whycon/poses", 10, &DroneControl::marker_position_cb, drone_control);
  local_pos_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &DroneControl::local_position_cb, drone_control);
  svo_pos_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/svo/pose_imu", 10, &DroneControl::svo_position_cb, drone_control);

  setpoint_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  vision_pos_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
  svo_cmd_pub_ = nh.advertise<std_msgs::String>("/svo/remote_key", 10);

  arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  land_client_ = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  // The tf module of mavros does not work currently
  // /mavros/setpoint_position/tf/ could also be used in approachMarker function
  //nh.setParam("/mavros/local_position/tf/frame_id", "map");
  //nh.setParam("/mavros/local_position/tf/child_frame_id", "drone");
  //nh.setParam("/mavros/local_position/tf/send", true);
}
