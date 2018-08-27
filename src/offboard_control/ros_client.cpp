#include "include/ros_client.h"

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>

ROSClient::ROSClient(int &argc, char **argv)
{
  ros::init(argc, argv, "offboard_ctrl");
  this->nh_ = new ros::NodeHandle();

  avoidCollision_ = false;
}

void ROSClient::init(DroneControl *const drone_control)
{
  state_sub_ = nh_->subscribe<mavros_msgs::State>("/mavros/state", 10, &DroneControl::state_cb, drone_control);
  extended_state_sub_ = nh_->subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state", 10, &DroneControl::extended_state_cb, drone_control);
  marker_pos_sub_ = nh_->subscribe<geometry_msgs::PoseArray>("/whycon/poses", 10, &DroneControl::marker_position_cb, drone_control);
  local_pos_sub_ = nh_->subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &DroneControl::local_position_cb, drone_control);
  global_pos_sub_ = nh_->subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, &DroneControl::global_position_cb, drone_control);
  svo_pos_sub_ = nh_->subscribe<geometry_msgs::PoseWithCovarianceStamped>("/svo/pose_imu", 10, &DroneControl::svo_position_cb, drone_control);
  setpoint_pos_sub_ = nh_->subscribe<geometry_msgs::PoseStamped>("/trajectory/setpoint_position", 10, &DroneControl::setpoint_position_cb, drone_control);

  global_setpoint_pos_pub_ = nh_->advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_position/global", 10);
  setpoint_pos_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  endpoint_pos_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("/trajectory/endpoint_position", 10);
  vision_pos_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
  svo_cmd_pub_ = nh_->advertise<std_msgs::String>("/svo/remote_key", 10);
  ewok_cmd_pub_ = nh_->advertise<std_msgs::String>("/trajectory/command", 10);

  arming_client_ = nh_->serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  land_client_ = nh_->serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  set_mode_client_ = nh_->serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

  // The tf module of mavros does not work currently
  // /mavros/setpoint_position/tf/ could also be used in approachMarker function
  //nh_->setParam("/mavros/local_position/tf/frame_id", "map");
  //nh_->setParam("/mavros/local_position/tf/child_frame_id", "drone");
  //nh_->setParam("/mavros/local_position/tf/send", true);
}

void ROSClient::publishTrajectoryEndpoint(const geometry_msgs::PoseStamped &setpoint_pos_ENU)
{
  if(avoidCollision_)
  {
    endpoint_pos_pub_.publish(setpoint_pos_ENU);
    ros::spinOnce();
  }
  else
  {
    ROS_INFO("Collision avoidance has not been enabled");
  }
}

void ROSClient::setParam(const std::string &key, double d)
{
  nh_->setParam(key, d);
}
