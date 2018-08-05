#include "include/drone_control.h"

#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <math.h>

DroneControl::DroneControl(ROSClient *ros_client)
{
  this->ros_client_ = ros_client;
  this->ros_client_->init(this);

  // The setpoint publishing rate MUST be faster than 2Hz
  this->rate_ = new ros::Rate(ROS_RATE);

  static tf2_ros::TransformListener tfListener(tfBuffer_);
}

void DroneControl::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  current_state_ = *msg;
}

void DroneControl::marker_position_cb(const geometry_msgs::PoseArray::ConstPtr &msg)
{
  marker_position_ = *msg;

  static tf2_ros::TransformBroadcaster br;

  // Transformation from drone to visual marker
  transformStamped_.header.stamp = ros::Time::now();
  transformStamped_.header.frame_id = "drone";
  transformStamped_.child_frame_id = "marker";
  transformStamped_.transform.translation.x = marker_position_.poses[0].position.z;
  transformStamped_.transform.translation.y = -marker_position_.poses[0].position.x;
  transformStamped_.transform.translation.z = -marker_position_.poses[0].position.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transformStamped_.transform.rotation.x = q.x();
  transformStamped_.transform.rotation.y = q.y();
  transformStamped_.transform.rotation.z = q.z();
  transformStamped_.transform.rotation.w = q.w();
  br.sendTransform(transformStamped_);

  float target_distance = marker_position_.poses[0].position.z/4; // Target distance is proportional to horizontal distance
  if (target_distance < 1) target_distance = 1; // Minimum of 1 meter

  // Transformation from visual marker to target position
  transformStamped_.header.stamp = ros::Time::now();
  transformStamped_.header.frame_id = "marker";
  transformStamped_.child_frame_id = "target_position";
  if (close_enough_ > (SAFETY_TIME_SEC * ROS_RATE))
    transformStamped_.transform.translation.x = -0.6; //The target is 0.6 m in front of the marker if the drone is close enough
  else
    transformStamped_.transform.translation.x = -target_distance;
  transformStamped_.transform.translation.y = 0;
  transformStamped_.transform.translation.z = 0;
  transformStamped_.transform.rotation = marker_position_.poses[0].orientation;
  br.sendTransform(transformStamped_);

  if (approaching_)
  {
    try
    {
      transformStamped_ = tfBuffer_.lookupTransform("map", "target_position", ros::Time(0));
      setpoint_pos_ENU_.pose.position.x = transformStamped_.transform.translation.x;
      setpoint_pos_ENU_.pose.position.y = transformStamped_.transform.translation.y;
      setpoint_pos_ENU_.pose.position.z = transformStamped_.transform.translation.z;
      //setpoint_pos_ENU.pose.orientation = tf::createQuaternionMsgFromYaw(currentYaw());

      ROS_INFO("Setpoint position: E: %f, N: %f, U: %f", transformStamped_.transform.translation.x,
               transformStamped_.transform.translation.y, transformStamped_.transform.translation.z);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
    }
  }
}

void DroneControl::local_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  local_position_ = *msg;
  static int cnt = 0;

  static tf2_ros::TransformBroadcaster br;

  // Transformation from map to drone
  transformStamped_.header.stamp = ros::Time::now();
  transformStamped_.header.frame_id = "map";
  transformStamped_.child_frame_id = "drone";
  transformStamped_.transform.translation.x = local_position_.pose.position.x;
  transformStamped_.transform.translation.y = local_position_.pose.position.y;
  transformStamped_.transform.translation.z = local_position_.pose.position.z;
  transformStamped_.transform.rotation = local_position_.pose.orientation;

  cnt++;
  if (cnt % 100 == 0) \
  {
    ROS_INFO("Mavros local position: E: %f, N: %f, U: %f, yaw: %f", transformStamped_.transform.translation.x,
             transformStamped_.transform.translation.y, transformStamped_.transform.translation.z, currentYaw());
  }

  br.sendTransform(transformStamped_);
}

void DroneControl::svo_position_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  svo_position_ = *msg;
  static int cnt = 0;

  static tf2_ros::TransformBroadcaster br;
  static tf2_ros::StaticTransformBroadcaster sbr;

  if (ros::Time::now() - last_svo_estimate_ > ros::Duration(1.0))
  {
    // svo_position is the first pose message after initialization/recovery, need to set svo_init_pos
    if (ros::Time::now() - local_position_.header.stamp < ros::Duration(1.0))
    {
      ROS_INFO("svo_init_pos = local_position");
      svo_init_pos_ = local_position_;
    }
    else
    {
      ROS_INFO("svo_init_pos = 0");
      svo_init_pos_.pose.position.x = 0;
      svo_init_pos_.pose.position.y = 0;
      svo_init_pos_.pose.position.z = 0;
      svo_init_pos_.pose.orientation.x = 0;
      svo_init_pos_.pose.orientation.y = 0;
      svo_init_pos_.pose.orientation.z = 0;
      svo_init_pos_.pose.orientation.w = 1;
    }

    // Transformation from map to svo_init
    transformStamped_.header.stamp = ros::Time::now();
    transformStamped_.header.frame_id = "map";
    transformStamped_.child_frame_id = "svo_init";
    transformStamped_.transform.translation.x = svo_init_pos_.pose.position.x;
    transformStamped_.transform.translation.y = svo_init_pos_.pose.position.y;
    transformStamped_.transform.translation.z = svo_init_pos_.pose.position.z;
    transformStamped_.transform.rotation = svo_init_pos_.pose.orientation;

    sbr.sendTransform(transformStamped_);
  }

  // Transformation from svo_init to drone_vision
  transformStamped_.header.stamp = last_svo_estimate_ = ros::Time::now();
  transformStamped_.header.frame_id = "svo_init";
  transformStamped_.child_frame_id = "drone_vision";
  transformStamped_.transform.translation.x = svo_position_.pose.pose.position.x;
  transformStamped_.transform.translation.y = svo_position_.pose.pose.position.y;
  transformStamped_.transform.translation.z = svo_position_.pose.pose.position.z;
  transformStamped_.transform.rotation = svo_position_.pose.pose.orientation;

  br.sendTransform(transformStamped_);

  if (send_vision_estimate_)
  {
    try
    {
      // Send vision position estimate to mavros
      transformStamped_ = tfBuffer_.lookupTransform("map", "drone_vision", ros::Time(0));
      vision_pos_ENU_.header.stamp = ros::Time::now();
      vision_pos_ENU_.header.frame_id = "map";
      vision_pos_ENU_.pose.position.x = transformStamped_.transform.translation.x;
      vision_pos_ENU_.pose.position.y = transformStamped_.transform.translation.y;
      vision_pos_ENU_.pose.position.z = transformStamped_.transform.translation.z;
      vision_pos_ENU_.pose.orientation = transformStamped_.transform.rotation;

      ros_client_->vision_pos_pub_.publish(vision_pos_ENU_);

      cnt++;
      if (cnt % 66 == 0)
      {
        double roll, pitch, yaw;
        tf::Quaternion q(vision_pos_ENU_.pose.orientation.x,
                         vision_pos_ENU_.pose.orientation.y,
                         vision_pos_ENU_.pose.orientation.z,
                         vision_pos_ENU_.pose.orientation.w);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        ROS_INFO("Vision position lookup: E: %f, N: %f, U: %f, yaw: %f", transformStamped_.transform.translation.x,
                 transformStamped_.transform.translation.y, transformStamped_.transform.translation.z, (float)yaw);
      }
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
    }
  }
}

void DroneControl::offboardMode()
{
  // Wait for FCU connection
  while(ros::ok() && current_state_.connected)
  {
    ros::spinOnce();
    rate_->sleep();
    ROS_INFO("connecting to FCU...");
  }

  ROS_INFO("Switching to OFFBOARD mode");

  last_svo_estimate_ = ros::Time::now(); //TODO this is error prone

  if (ros::Time::now() - local_position_.header.stamp < ros::Duration(1.0)) {
      ROS_INFO("Local_position available");
  } else {
      ROS_INFO("Local_position not available, initializing to 0");
      local_position_.header.stamp = ros::Time::now();
      local_position_.header.frame_id = "map";
      local_position_.pose.position.x = 0;
      local_position_.pose.position.y = 0;
      local_position_.pose.position.z = 0;
      local_position_.pose.orientation.x = 0;
      local_position_.pose.orientation.y = 0;
      local_position_.pose.orientation.z = 0;
      local_position_.pose.orientation.w = 1;
  }

  setpoint_pos_ENU_ = gps_init_pos_ = local_position_;


  // Send a few setpoints before starting, otherwise px4 will not switch to OFFBOARD mode
  for(int i = 20; ros::ok() && i > 0; --i)
  {
    ros_client_->publishSetpoint(setpoint_pos_ENU_);
    rate_->sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  arm_cmd_.request.value = true;

  last_request_ = ros::Time::now();

  // Change to offboard mode and arm
  while(ros::ok() && !current_state_.armed)
  {
    if( current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0)))
    {
      ROS_INFO(current_state_.mode.c_str());
      if( ros_client_->set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request_ = ros::Time::now();
    }
    else
    {
      if( !current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0)))
      {
        if( ros_client_->arming_client_.call(arm_cmd_) && arm_cmd_.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    }
    ros_client_->publishSetpoint(setpoint_pos_ENU_);
    rate_->sleep();
  }

  return;
}

void DroneControl::vioOff()
{
  ROS_INFO("Disabling SVO");

  svo_cmd_.data = "r";
  for(int i = 0; ros::ok() && i < 1 * ROS_RATE; ++i)
  {
    ros_client_->svo_cmd_pub_.publish(svo_cmd_);
    ros::spinOnce();
    rate_->sleep();
  }

  return;
}

void DroneControl::vioOn()
{
  ROS_INFO("Starting SVO");

  svo_cmd_.data = "s";
  for(int i = 0; ros::ok() && i < 1 * ROS_RATE; ++i)
  {
    ros_client_->svo_cmd_pub_.publish(svo_cmd_);
    ros::spinOnce();
    rate_->sleep();
  }

  return;
}

void DroneControl::takeOff()
{
  ROS_INFO("Taking off. Current position: N: %f, W: %f, U: %f", local_position_.pose.position.x,
           local_position_.pose.position.y, local_position_.pose.position.z);

  // Take off
  setpoint_pos_ENU_ = gps_init_pos_;
  setpoint_pos_ENU_.pose.position.z += FLIGHT_ALTITUDE;

  ROS_INFO("Taking off");
  for(int i = 0; ros::ok() && i < 10 * ROS_RATE; ++i)
  {
    ros_client_->publishSetpoint(setpoint_pos_ENU_);
    rate_->sleep();
  }
  ROS_INFO("Takeoff finished! Looking for whycon marker");
  return;
}

void DroneControl::initVIO()
{
  vioOn();

  //Translational movement to start odometry
  for(int j = 0; ros::ok() && j < INIT_FLIGHT_REPEAT && ros::Time::now() - last_svo_estimate_ > ros::Duration(1.0); ++j)
  {
    ROS_INFO("Translational movement");
    for(int i = 0; ros::ok() && i < INIT_FLIGHT_DURATION * ROS_RATE
        && ros::Time::now() - last_svo_estimate_ > ros::Duration(1.0); ++i)
    {

      setpoint_pos_ENU_.pose.position.x += INIT_FLIGHT_LENGTH/INIT_FLIGHT_DURATION/ROS_RATE;

      ros_client_->publishSetpoint(setpoint_pos_ENU_);
      rate_->sleep();
    }
    for(int i = 0; ros::ok() && i < INIT_FLIGHT_DURATION * ROS_RATE
        && ros::Time::now() - last_svo_estimate_ > ros::Duration(1.0); ++i)
    {

      setpoint_pos_ENU_.pose.position.x -= INIT_FLIGHT_LENGTH/INIT_FLIGHT_DURATION/ROS_RATE;

      ros_client_->publishSetpoint(setpoint_pos_ENU_);
      rate_->sleep();
    }
  }

  if (ros::Time::now() - last_svo_estimate_ < ros::Duration(1.0))
  {
    ROS_INFO("SVO initialized successfully");
    svo_running_ = true;
  }
  else
    ROS_INFO("SVO initialization failed");

  setpoint_pos_ENU_ = gps_init_pos_;
  setpoint_pos_ENU_.pose.position.z += FLIGHT_ALTITUDE;

  ROS_INFO("Back to takeoff position");
  for(int i = 0; ros::ok() && i < 10 * ROS_RATE; ++i)
  {
    ros_client_->publishSetpoint(setpoint_pos_ENU_);
    rate_->sleep();
  }


  return;
}

void DroneControl::testFlightHorizontal()
{
  ROS_INFO("Horizontal test flight");

  for(int j = 0; ros::ok() && j < TEST_FLIGHT_REPEAT; ++j)
  {
    for(int i = 0; ros::ok() && i < TEST_FLIGHT_DURATION * ROS_RATE; ++i)
    {
      setpoint_pos_ENU_.pose.position.x += TEST_FLIGHT_LENGTH/TEST_FLIGHT_DURATION/ROS_RATE;

      ros_client_->publishSetpoint(setpoint_pos_ENU_);
      rate_->sleep();
    }
    for(int i = 0; ros::ok() && i < TEST_FLIGHT_DURATION * ROS_RATE; ++i)
    {
      setpoint_pos_ENU_.pose.position.y += TEST_FLIGHT_LENGTH/TEST_FLIGHT_DURATION/ROS_RATE;

      ros_client_->publishSetpoint(setpoint_pos_ENU_);
      rate_->sleep();
    }
    for(int i = 0; ros::ok() && i < TEST_FLIGHT_DURATION * ROS_RATE; ++i)
    {
      setpoint_pos_ENU_.pose.position.x -= TEST_FLIGHT_LENGTH/TEST_FLIGHT_DURATION/ROS_RATE;

      ros_client_->publishSetpoint(setpoint_pos_ENU_);
      rate_->sleep();
    }
    for(int i = 0; ros::ok() && i < TEST_FLIGHT_DURATION * ROS_RATE; ++i)
    {
      setpoint_pos_ENU_.pose.position.y -= TEST_FLIGHT_LENGTH/TEST_FLIGHT_DURATION/ROS_RATE;

      ros_client_->publishSetpoint(setpoint_pos_ENU_);
      rate_->sleep();
    }
  }
}

void DroneControl::testFlightVertical()
{
  ROS_INFO("Vertical test flight");

  for(int j = 0; ros::ok() && j < TEST_FLIGHT_REPEAT; ++j)
  {
    for(int i = 0; ros::ok() && i < TEST_FLIGHT_DURATION * ROS_RATE; ++i)
    {
      setpoint_pos_ENU_.pose.position.x += TEST_FLIGHT_LENGTH/TEST_FLIGHT_DURATION/ROS_RATE;

      ros_client_->publishSetpoint(setpoint_pos_ENU_);
      rate_->sleep();
    }
    for(int i = 0; ros::ok() && i < TEST_FLIGHT_DURATION * ROS_RATE; ++i)
    {
      setpoint_pos_ENU_.pose.position.z += TEST_FLIGHT_LENGTH/TEST_FLIGHT_DURATION/ROS_RATE;

      ros_client_->publishSetpoint(setpoint_pos_ENU_);
      rate_->sleep();
    }
    for(int i = 0; ros::ok() && i < TEST_FLIGHT_DURATION * ROS_RATE; ++i)
    {
      setpoint_pos_ENU_.pose.position.x -= TEST_FLIGHT_LENGTH/TEST_FLIGHT_DURATION/ROS_RATE;

      ros_client_->publishSetpoint(setpoint_pos_ENU_);
      rate_->sleep();
    }
    for(int i = 0; ros::ok() && i < TEST_FLIGHT_DURATION * ROS_RATE; ++i)
    {
      setpoint_pos_ENU_.pose.position.z -= TEST_FLIGHT_LENGTH/TEST_FLIGHT_DURATION/ROS_RATE;

      ros_client_->publishSetpoint(setpoint_pos_ENU_);
      rate_->sleep();
    }
  }
}

void DroneControl::hover(int seconds)
{
  ROS_INFO("Hovering for %i seconds in position: E: %f, N: %f, U: %f", seconds,
           setpoint_pos_ENU_.pose.position.x,
           setpoint_pos_ENU_.pose.position.y,
           setpoint_pos_ENU_.pose.position.z);
  for(int i = 0; ros::ok() && i < 15 * ROS_RATE; ++i)
  {
    ros_client_->publishSetpoint(setpoint_pos_ENU_);
    rate_->sleep();
  }

  return;
}

void DroneControl::turnTowardsMarker()
{
  float rad, current_yaw;

  // Turn towards the marker without change of position
  setpoint_pos_ENU_.pose.position.x = local_position_.pose.position.x;
  setpoint_pos_ENU_.pose.position.y = local_position_.pose.position.y;
  setpoint_pos_ENU_.pose.position.z = local_position_.pose.position.z;

  for(int j = 0; ros::ok() && j < 10 * ROS_RATE; ++j)
  {
    current_yaw = currentYaw();

    if (ros::Time::now() - marker_position_.header.stamp < ros::Duration(1.0))
    {
      // Calculate yaw angle difference of marker in radians
      rad = -atan2f(marker_position_.poses[0].position.x, marker_position_.poses[0].position.z);
      if (fabs(rad) < 0.1)
      {
        ROS_INFO("Headed towards marker!");
        break;
      }

      ROS_INFO("Marker found, current yaw: %f, turning %f radians", current_yaw, rad);
      setpoint_pos_ENU_.pose.orientation = tf::createQuaternionMsgFromYaw(current_yaw+rad);
    }
    else
    {
      ROS_INFO("No marker was found in the last second, turning around");
      setpoint_pos_ENU_.pose.orientation = tf::createQuaternionMsgFromYaw(current_yaw+TURN_STEP_RAD);
    }
    ros_client_->publishSetpoint(setpoint_pos_ENU_);
    rate_->sleep();
  }

  // Send setpoint for 2 seconds
  for(int i = 0; ros::ok() && i < 2 * ROS_RATE; ++i)
  {
    ros_client_->publishSetpoint(setpoint_pos_ENU_);
    rate_->sleep();
  }

  return;
}

void DroneControl::approachMarker()
{
  approaching_ = true;

  // TODO: handle after MAX_ATTEMPTS
  for(int j = 0; ros::ok() && j < MAX_ATTEMPTS; ++j)
  {
    if (ros::Time::now() - marker_position_.header.stamp < ros::Duration(1.0))
    {
      if (marker_position_.poses[0].position.z < 1.5)
      {
        close_enough_++;
        // TODO: Changing orientation. Calulate yaw from marker orientation

        if (close_enough_ > (SAFETY_TIME_SEC * ROS_RATE))
        {
          ROS_INFO("Close enough");
          break; // Exit loop and fly to final target
        }
      }
      else {close_enough_ = 0;}

      ros_client_->publishSetpoint(setpoint_pos_ENU_);

      approaching_ = true;

    }
    else
    {
      approaching_ = false;
      ROS_INFO("No marker was found in the last 1 second");
    }
    ros::spinOnce();
    rate_->sleep();
  }

  // Publish final setpoint for 4 seconds before landing
  for(int i = 0; ros::ok() && i < 4 * ROS_RATE; ++i)
  {
    ros_client_->publishSetpoint(setpoint_pos_ENU_);
    rate_->sleep();
  }

  approaching_ = false;
  ROS_INFO("Marker approached!");

  return;
}

void DroneControl::land()
{
  mavros_msgs::CommandTOL land_cmd;
  land_cmd.request.yaw = 0;
  land_cmd.request.latitude = 0;
  land_cmd.request.longitude = 0;
  land_cmd.request.altitude = 0;

  ROS_INFO("Trying to land");
  while (!(ros_client_->land_client_.call(land_cmd) && land_cmd.response.success))
  {
    ros_client_->publishSetpoint(setpoint_pos_ENU_);
    ROS_INFO("Retrying to land");
    rate_->sleep();
  }
  ROS_INFO("Success");

  // Wait 5 second for proper landing
  for(int i = 0; ros::ok() && i < 5 * ROS_RATE; ++i)
  {
    ros::spinOnce();
    rate_->sleep();
  }

  return;
}

void DroneControl::disarm()
{
  // Disarm
  arm_cmd_.request.value = false;
  while(ros::ok() && current_state_.armed)
  {
    if( current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0)))
    {
      if( ros_client_->arming_client_.call(arm_cmd_) && arm_cmd_.response.success)
      {
        ROS_INFO("Vehicle disarmed");
      }
      last_request_ = ros::Time::now();
    }
    ros::spinOnce();
    rate_->sleep();
  }
  return;
}

float DroneControl::currentYaw()
{
  //Calculate yaw current orientation
  double roll, pitch, yaw;
  tf::Quaternion q(local_position_.pose.orientation.x,
                   local_position_.pose.orientation.y,
                   local_position_.pose.orientation.z,
                   local_position_.pose.orientation.w);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  return yaw;
}
