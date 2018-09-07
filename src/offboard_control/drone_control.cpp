#include "include/drone_control.h"

#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

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

void DroneControl::extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr &msg)
{
  landed_state_ = msg->landed_state;
}

void DroneControl::marker_position_cb(const geometry_msgs::PoseArray::ConstPtr &msg)
{
  marker_position_ = *msg;
  static int cnt = 0;

  static tf2_ros::TransformBroadcaster br;

  // Transformation from drone to visual marker
  transformStamped_.header.stamp = marker_position_.header.stamp;
  transformStamped_.header.frame_id = "drone";
  transformStamped_.child_frame_id = "marker";
  transformStamped_.transform.translation.x = marker_position_.poses[0].position.z;
  transformStamped_.transform.translation.y = -marker_position_.poses[0].position.x;
  transformStamped_.transform.translation.z = -marker_position_.poses[0].position.y;
  if(USE_MARKER_ORIENTATION)
  {
      // Calculate yaw difference between drone and marker orientation
      double yaw = getYaw(marker_position_.poses[0].orientation);
      if(yaw < -M_PI/2) yaw += M_PI;
      else if(yaw > M_PI/2) yaw -= M_PI;
      //else if(yaw < -3*M_PI/2) yaw += 2*M_PI;
      //else if(yaw > 3*M_PI/2) yaw -= 2*M_PI;
      transformStamped_.transform.rotation = tf::createQuaternionMsgFromYaw(yaw);
  }
  else
  {
    double rad = -atan2f(marker_position_.poses[0].position.x, marker_position_.poses[0].position.z);
    transformStamped_.transform.rotation = tf::createQuaternionMsgFromYaw(rad);
  }
  br.sendTransform(transformStamped_);

  double target_distance = marker_position_.poses[0].position.z/4; // Target distance is proportional to horizontal distance
  if(target_distance < 1) target_distance = 1; // Minimum of 1 meter

  // Transformation from visual marker to target position
  transformStamped_.header.stamp = marker_position_.header.stamp;
  transformStamped_.header.frame_id = "marker";
  transformStamped_.child_frame_id = "target_position";
  if(close_enough_ > (SAFETY_TIME_SEC * ROS_RATE) || ros_client_->avoidCollision_)
    transformStamped_.transform.translation.x = -0.4; //The target is 0.4 m in front of the marker if the drone is close enough or collision avoidance is active
  else
    transformStamped_.transform.translation.x = -target_distance;
  transformStamped_.transform.translation.y = 0;
  transformStamped_.transform.translation.z = 0;
  transformStamped_.transform.rotation.x = 0;
  transformStamped_.transform.rotation.y = 0;
  transformStamped_.transform.rotation.z = 0;
  transformStamped_.transform.rotation.w = 1;
  br.sendTransform(transformStamped_);

  if(approaching_)
  {
    try
    {
      transformStamped_ = tfBuffer_.lookupTransform("world", "target_position", ros::Time(0));

      endpoint_pos_ENU_.pose.position.x = transformStamped_.transform.translation.x;
      endpoint_pos_ENU_.pose.position.y = transformStamped_.transform.translation.y;
      endpoint_pos_ENU_.pose.position.z = transformStamped_.transform.translation.z;
      double yaw = getYaw(transformStamped_.transform.rotation);
      endpoint_pos_ENU_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

      endpoint_active_ = true;

      if(cnt % 66 == 0)
      {
        ROS_INFO("Endpoint position: E: %f, N: %f, U: %f, yaw: %f", transformStamped_.transform.translation.x,
                transformStamped_.transform.translation.y, transformStamped_.transform.translation.z, yaw);
      }
      cnt++;
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
  static tf2_ros::StaticTransformBroadcaster sbr;

  // Transformation from world to drone
  transformStamped_.header.stamp = local_position_.header.stamp;
  transformStamped_.header.frame_id = "world";
  transformStamped_.child_frame_id = "drone";
  transformStamped_.transform.translation.x = local_position_.pose.position.x;
  transformStamped_.transform.translation.y = local_position_.pose.position.y;
  transformStamped_.transform.translation.z = local_position_.pose.position.z;
  transformStamped_.transform.rotation = local_position_.pose.orientation;
  br.sendTransform(transformStamped_);

  if(!cam_tf_init_)
  {
    // Transformation from drone to camera
    transformStamped_.header.stamp = ros::Time::now();
    transformStamped_.header.frame_id = "drone";
    transformStamped_.child_frame_id = "camera";
    transformStamped_.transform.translation.x = 0.1;
    transformStamped_.transform.translation.y = 0;
    transformStamped_.transform.translation.z = 0;
    transformStamped_.transform.rotation.x = -0.5;
    transformStamped_.transform.rotation.y = 0.5;
    transformStamped_.transform.rotation.z = -0.5;
    transformStamped_.transform.rotation.w = 0.5;
    sbr.sendTransform(transformStamped_);

    ROS_INFO("Drone to camera transform initialized");
    cam_tf_init_ = true;
  }

  cnt++;
  if(cnt % 100 == 0)
  {
    ROS_INFO("Mavros local position: E: %f, N: %f, U: %f, yaw: %f", transformStamped_.transform.translation.x,
             transformStamped_.transform.translation.y, transformStamped_.transform.translation.z, currentYaw());
  }
}

void DroneControl::global_position_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
  global_position_ = *msg;
  static int cnt = 0;

  cnt++;
  if(cnt % 100 == 0)
  {
    //ROS_INFO("GPS: lat: %f, long: %f, alt: %f", msg->latitude, msg->longitude, msg->altitude);
  }
}

void DroneControl::setpoint_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  if(approaching_ && !endpoint_active_) return;
  else setpoint_pos_ENU_ = *msg;
}

void DroneControl::svo_position_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  svo_position_ = *msg;
  static int cnt = 0;

  static tf2_ros::TransformBroadcaster br;
  static tf2_ros::StaticTransformBroadcaster sbr;

  if(ros::Time::now() - last_svo_estimate_ > ros::Duration(1.0))
  {
    // svo_position is the first pose message after initialization/recovery, need to set svo_init_pos
    if(ros::Time::now() - local_position_.header.stamp < ros::Duration(0.5))
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

    // Transformation from world to svo_init
    transformStamped_.header.stamp = svo_position_.header.stamp;
    transformStamped_.header.frame_id = "world";
    transformStamped_.child_frame_id = "svo_init";
    transformStamped_.transform.translation.x = svo_init_pos_.pose.position.x;
    transformStamped_.transform.translation.y = svo_init_pos_.pose.position.y;
    transformStamped_.transform.translation.z = svo_init_pos_.pose.position.z;
    transformStamped_.transform.rotation = svo_init_pos_.pose.orientation;

    sbr.sendTransform(transformStamped_);
  }

  // Transformation from svo_init to drone_vision
  transformStamped_.header.stamp = last_svo_estimate_ = svo_position_.header.stamp;
  transformStamped_.header.frame_id = "svo_init";
  transformStamped_.child_frame_id = "drone_vision";
  transformStamped_.transform.translation.x = svo_position_.pose.pose.position.x;
  transformStamped_.transform.translation.y = svo_position_.pose.pose.position.y;
  transformStamped_.transform.translation.z = svo_position_.pose.pose.position.z;
  transformStamped_.transform.rotation = svo_position_.pose.pose.orientation;

  br.sendTransform(transformStamped_);

  if(send_vision_estimate_)
  {
    try
    {
      // Send vision position estimate to mavros
      transformStamped_ = tfBuffer_.lookupTransform("world", "drone_vision", ros::Time(0));
      vision_pos_ENU_.header.stamp = svo_position_.header.stamp;
      vision_pos_ENU_.header.frame_id = "world";
      vision_pos_ENU_.pose.position.x = transformStamped_.transform.translation.x;
      vision_pos_ENU_.pose.position.y = transformStamped_.transform.translation.y;
      vision_pos_ENU_.pose.position.z = transformStamped_.transform.translation.z;
      vision_pos_ENU_.pose.orientation = transformStamped_.transform.rotation;

      ros_client_->vision_pos_pub_.publish(vision_pos_ENU_);

      cnt++;
      if(cnt % 66 == 0)
      {
        ROS_INFO("Vision position lookup: E: %f, N: %f, U: %f, yaw: %f", transformStamped_.transform.translation.x,
                 transformStamped_.transform.translation.y, transformStamped_.transform.translation.z, getYaw(transformStamped_.transform.rotation));
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

  // Wait for ROS
  for(int i = 0; ros::ok() && i < 4 * ROS_RATE; ++i)
  {
    ros::spinOnce();
    rate_->sleep();
  }

  ROS_INFO("Switching to OFFBOARD mode");

  last_svo_estimate_ = ros::Time::now(); //TODO this is error prone

  if(ros::Time::now() - local_position_.header.stamp < ros::Duration(1.0))
  {
    ROS_INFO("Local_position available");
  }
  else
  {
    ROS_WARN("Local_position not available, initializing to 0");
    local_position_.header.stamp = ros::Time::now();
    local_position_.header.frame_id = "world";
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
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
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
      ROS_INFO("%s",current_state_.mode.c_str());
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
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    rate_->sleep();
  }

  return;
}

void DroneControl::vioOff()
{
  ROS_INFO("Disabling SVO");

  svo_cmd_.data = "r";

  ros_client_->svo_cmd_pub_.publish(svo_cmd_);
  ros::spinOnce();
  rate_->sleep();

  return;
}

void DroneControl::vioOn()
{
  //double height = local_position_.pose.position.z;
  //ros_client_->setParam("/svo/map_scale", height); //TODO: this is not having any effect, parameter is probably read at startup
  //ROS_INFO("Starting SVO at height %f", height);
  //ros::spinOnce();
  //hover(5.0); //Wait fot the parameter change to be processed

  ROS_INFO("Starting SVO");

  svo_cmd_.data = "s";

  ros_client_->svo_cmd_pub_.publish(svo_cmd_);
  ros::spinOnce();

  return;
}

void DroneControl::collisionAvoidOff()
{
  ROS_INFO("Disabling collision avoidance");

  ros_client_->avoidCollision_ = false;

  ewok_cmd_.data = "r";
  ros_client_->ewok_cmd_pub_.publish(ewok_cmd_);

  return;
}

void DroneControl::collisionAvoidOn()
{
  ROS_INFO("Starting collision avoidance");

  ros_client_->avoidCollision_ = true;

  ewok_cmd_.data = "s";
  ros_client_->ewok_cmd_pub_.publish(ewok_cmd_);

  return;
}

void DroneControl::takeOff()
{
  ROS_INFO("Taking off. Current position: E: %f, N: %f, U: %f", local_position_.pose.position.x,
           local_position_.pose.position.y, local_position_.pose.position.z);

  // Take off
  setpoint_pos_ENU_ = gps_init_pos_;
  setpoint_pos_ENU_.pose.position.z += TAKEOFF_ALTITUDE;

  ROS_INFO("Taking off");
  for(int i = 0; ros::ok() && i < 10 * ROS_RATE; ++i)
  {
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    rate_->sleep();
  }
  ROS_INFO("Takeoff finished!");
  return;
}

void DroneControl::initVIO()
{
  vioOn();

  //Translational movement to start odometry
  for(int j = 0; ros::ok() && j < INIT_FLIGHT_REPEAT
      && ros::Time::now() - last_svo_estimate_ > ros::Duration(1.0); ++j)
  {
    ROS_INFO("Translational movement");
    for(int i = 0; ros::ok() && i < INIT_FLIGHT_DURATION * ROS_RATE
        && ros::Time::now() - last_svo_estimate_ > ros::Duration(1.0); ++i)
    {
      setpoint_pos_ENU_.pose.position.x += INIT_FLIGHT_LENGTH/INIT_FLIGHT_DURATION/ROS_RATE;

      ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
      ros::spinOnce();
      rate_->sleep();
    }
    for(int i = 0; ros::ok() && i < INIT_FLIGHT_DURATION * ROS_RATE
        && ros::Time::now() - last_svo_estimate_ > ros::Duration(1.0); ++i)
    {
      setpoint_pos_ENU_.pose.position.x -= INIT_FLIGHT_LENGTH/INIT_FLIGHT_DURATION/ROS_RATE;

      ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
      ros::spinOnce();
      rate_->sleep();
    }
  }

  if(ros::Time::now() - last_svo_estimate_ < ros::Duration(1.0))
  {
    ROS_INFO("SVO initialized successfully");
    svo_running_ = true;
  }
  else
    ROS_INFO("SVO initialization failed");

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

      ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
      ros::spinOnce();
      rate_->sleep();
    }
    for(int i = 0; ros::ok() && i < TEST_FLIGHT_DURATION * ROS_RATE; ++i)
    {
      setpoint_pos_ENU_.pose.position.y += TEST_FLIGHT_LENGTH/TEST_FLIGHT_DURATION/ROS_RATE;

      ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
      ros::spinOnce();
      rate_->sleep();
    }
    for(int i = 0; ros::ok() && i < TEST_FLIGHT_DURATION * ROS_RATE; ++i)
    {
      setpoint_pos_ENU_.pose.position.x -= TEST_FLIGHT_LENGTH/TEST_FLIGHT_DURATION/ROS_RATE;

      ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
      ros::spinOnce();
      rate_->sleep();
    }
    for(int i = 0; ros::ok() && i < TEST_FLIGHT_DURATION * ROS_RATE; ++i)
    {
      setpoint_pos_ENU_.pose.position.y -= TEST_FLIGHT_LENGTH/TEST_FLIGHT_DURATION/ROS_RATE;

      ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
      ros::spinOnce();
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

      ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
      ros::spinOnce();
      rate_->sleep();
    }
    for(int i = 0; ros::ok() && i < TEST_FLIGHT_DURATION * ROS_RATE; ++i)
    {
      setpoint_pos_ENU_.pose.position.z += TEST_FLIGHT_LENGTH/TEST_FLIGHT_DURATION/ROS_RATE;

      ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
      ros::spinOnce();
      rate_->sleep();
    }
    for(int i = 0; ros::ok() && i < TEST_FLIGHT_DURATION * ROS_RATE; ++i)
    {
      setpoint_pos_ENU_.pose.position.x -= TEST_FLIGHT_LENGTH/TEST_FLIGHT_DURATION/ROS_RATE;

      ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
      ros::spinOnce();
      rate_->sleep();
    }
    for(int i = 0; ros::ok() && i < TEST_FLIGHT_DURATION * ROS_RATE; ++i)
    {
      setpoint_pos_ENU_.pose.position.z -= TEST_FLIGHT_LENGTH/TEST_FLIGHT_DURATION/ROS_RATE;

      ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
      ros::spinOnce();
      rate_->sleep();
    }
  }
}

void DroneControl::flyToGlobal(double latitude, double longitude, double altitude, double yaw)
{
  mavros_msgs::GlobalPositionTarget target;
  target.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
  target.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_VX |
      mavros_msgs::GlobalPositionTarget::IGNORE_VY |
      mavros_msgs::GlobalPositionTarget::IGNORE_VZ |
      mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
      mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
      mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
      mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE;
  target.latitude = latitude;
  target.longitude = longitude;
  target.altitude = altitude;
  target.yaw = yaw;

  while(ros::ok() &&
         ( fabs(latitude - global_position_.latitude)*LAT_DEG_TO_M > 1.0
        || fabs(longitude - global_position_.longitude)*LON_DEG_TO_M > 1.0
        || fabs(altitude - global_position_.altitude) > 1.0))
  {
    ROS_INFO("Dist: lat: %f, long: %f, alt: %f", fabs(latitude - global_position_.latitude)*LAT_DEG_TO_M, fabs(longitude - global_position_.longitude)*LON_DEG_TO_M, altitude-global_position_.altitude);
    ros_client_->global_setpoint_pos_pub_.publish(target);
    ros::spinOnce();
    rate_->sleep();
  }
}

void DroneControl::flyToLocal(double x, double y, double z, double yaw)
{
  if(!std::isfinite(yaw))
  {
    yaw = currentYaw();
    ROS_INFO("Flying to local coordinates E: %f, N: %f, U: %f, current yaw: %f", x, y, z, yaw);
  }
  else ROS_INFO("Flying to local coordinates E: %f, N: %f, U: %f, yaw: %f", x, y, z, yaw);

  setpoint_pos_ENU_.pose.position.x = x;
  setpoint_pos_ENU_.pose.position.y = y;
  setpoint_pos_ENU_.pose.position.z = z;
  setpoint_pos_ENU_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  while(ros::ok() && distance(setpoint_pos_ENU_, local_position_) > 0.5)
  {
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    rate_->sleep();
  }

  //Publish for another second
  for(int i = 0; ros::ok() && i < 1 * ROS_RATE; ++i)
  {
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    rate_->sleep();
  }
}

void DroneControl::flyToLocalNoCollision(double x, double y, double z)
{
  int i, cnt = 0;

  geometry_msgs::PoseStamped endpoint;
  endpoint.pose.position.x = x;
  endpoint.pose.position.y = y;
  endpoint.pose.position.z = z;
  endpoint.pose.orientation = tf::createQuaternionMsgFromYaw(currentYaw());

  ros_client_->publishTrajectoryEndpoint(endpoint);

  for(i = 0; ros::ok() && cnt < 2 * ROS_RATE && i < 2*MAX_ATTEMPTS; ++i)
  {
    if(distance(endpoint, local_position_) < 0.5) cnt++;

    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    rate_->sleep();
  }
  if(i == MAX_ATTEMPTS) ROS_WARN("2*MAX_ATTEMPTS reached while flying to local coordinates. Aborting.");
}

void DroneControl::hover(double seconds)
{
  ROS_INFO("Hovering for %f seconds in position: E: %f, N: %f, U: %f", seconds,
           setpoint_pos_ENU_.pose.position.x,
           setpoint_pos_ENU_.pose.position.y,
           setpoint_pos_ENU_.pose.position.z);

  for(int i = 0; ros::ok() && i < 15 * ROS_RATE; ++i)
  {
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    rate_->sleep();
  }

  return;
}

void DroneControl::scanBuilding()
{
  ROS_INFO("Scanning building");
  double yaw = currentYaw();

  marker_found_ = false;
  geometry_msgs::PoseStamped current_endpoint = local_position_;

  //Fly down
  current_endpoint.pose.position.z = SAFETY_ALTITUDE_VIO;
  scanUntil(current_endpoint);

  //Fly 2 m to the right
  current_endpoint.pose.position.x += 2.0*sin(yaw);
  current_endpoint.pose.position.y -= 2.0*cos(yaw);
  scanUntil(current_endpoint);

  //Fly up
  current_endpoint.pose.position.z = SAFETY_ALTITUDE_GPS;
  scanUntil(current_endpoint);

  //Fly 4 m to the left
  current_endpoint.pose.position.x -= 4.0*sin(yaw);
  current_endpoint.pose.position.y += 4.0*cos(yaw);
  scanUntil(current_endpoint);

  //Fly down
  current_endpoint.pose.position.z = SAFETY_ALTITUDE_VIO;
  scanUntil(current_endpoint);

  return;
}

void DroneControl::scanUntil(const geometry_msgs::PoseStamped &endpoint)
{
  int i, cnt = 0;

  ros_client_->publishTrajectoryEndpoint(endpoint);

  for(i = 0; ros::ok() && cnt < 2 * ROS_RATE && !marker_found_ && i < 2*MAX_ATTEMPTS; ++i)
  {
    if(distance(endpoint, local_position_) < 0.5) cnt++;

    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    rate_->sleep();

    if(ros::Time::now() - marker_position_.header.stamp < ros::Duration(0.5))
    {
      marker_found_ = true;
    }
  }
  if(i == MAX_ATTEMPTS) ROS_WARN("2*MAX_ATTEMPTS reached while scanning building. Aborting.");
}

void DroneControl::centerMarker()
{
  // Center the marker without change of orientation
  int i, cnt = 0;
  double yaw = currentYaw();
  double verticalDistance = marker_position_.poses[0].position.z;

  try
  {
    transformStamped_ = tfBuffer_.lookupTransform("world", "marker", ros::Time(0));

    ROS_INFO("Marker is at: E: %f, N: %f, U: %f, yaw: %f", transformStamped_.transform.translation.x,
            transformStamped_.transform.translation.y, transformStamped_.transform.translation.z, yaw);
    endpoint_pos_ENU_.pose.position.x = transformStamped_.transform.translation.x - verticalDistance*cos(yaw);
    endpoint_pos_ENU_.pose.position.y = transformStamped_.transform.translation.y - verticalDistance*sin(yaw);
    endpoint_pos_ENU_.pose.position.z = transformStamped_.transform.translation.z;
    endpoint_pos_ENU_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    ROS_INFO("Centering at: E: %f, N: %f, U: %f, yaw: %f", endpoint_pos_ENU_.pose.position.x,
            endpoint_pos_ENU_.pose.position.y, endpoint_pos_ENU_.pose.position.z, yaw);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  ros_client_->publishTrajectoryEndpoint(endpoint_pos_ENU_);

  for(i = 0; ros::ok() && cnt < 2 * ROS_RATE && i < MAX_ATTEMPTS; ++i)
  {
    if(distance(endpoint_pos_ENU_, local_position_) < 0.5) cnt++;

    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    rate_->sleep();
  }
  if(i == MAX_ATTEMPTS) ROS_WARN("MAX_ATTEMPTS reached while centering marker. Aborting.");

  // Send setpoint for another second
  for(int i = 0; ros::ok() && i < 1 * ROS_RATE; ++i)
  {
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    rate_->sleep();
  }

  return;
}

void DroneControl::turnTowardsMarker()
{
  double rad, current_yaw;

  // Turn towards the marker without change of position
  setpoint_pos_ENU_ = local_position_;

  for(int j = 0; ros::ok() && j < 15 * ROS_RATE; ++j)
  {
    current_yaw = currentYaw();

    if(ros::Time::now() - marker_position_.header.stamp < ros::Duration(1.0))
    {
      // Calculate yaw angle difference of marker in radians
      rad = -atan2f(marker_position_.poses[0].position.x, marker_position_.poses[0].position.z);
      if(fabs(rad) < 0.1)
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
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    rate_->sleep();
  }

  // Send setpoint for 2 seconds
  for(int i = 0; ros::ok() && i < 2 * ROS_RATE; ++i)
  {
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    rate_->sleep();
  }

  return;
}

void DroneControl::approachMarker()
{
  int i, j, cnt = 0;
  approaching_ = true;

  // TODO: handle after MAX_ATTEMPTS
  for(j = 0; ros::ok() && j < MAX_ATTEMPTS; ++j)
  {
    if(ros::Time::now() - marker_position_.header.stamp < ros::Duration(1.0))
    {
      if(ros_client_->avoidCollision_)
      {
        for(i = 0; ros::ok() && !endpoint_active_ && i < MAX_ATTEMPTS; ++i)
        {
          ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
          ros::spinOnce();
          rate_->sleep();
        }
        if(i == MAX_ATTEMPTS)
        {
          ROS_WARN("MAX_ATTEMPTS reached while waiting for endpoint (while approaching marker). Aborting.");
          break;
        }
        ros_client_->publishTrajectoryEndpoint(endpoint_pos_ENU_);
        geometry_msgs::PoseStamped current_endpoint = endpoint_pos_ENU_;

        for(i = 0; ros::ok() && marker_position_.poses[0].position.z > 0.6 && i < MAX_ATTEMPTS; ++i)
        {
          if(distance(current_endpoint, endpoint_pos_ENU_) > marker_position_.poses[0].position.z/6.0)
          {
            ros_client_->publishTrajectoryEndpoint(endpoint_pos_ENU_);
            current_endpoint = endpoint_pos_ENU_;
          }
          ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
          ros::spinOnce();
          rate_->sleep();
        }
        if(i == MAX_ATTEMPTS)
        {
          ROS_WARN("MAX_ATTEMPTS reached while approaching marker in collision avoidance mode. Aborting.");
          break;
        }
        ROS_INFO("Close enough");
        break; // Exit loop and fly to final target
      }
      else
      {
        if(marker_position_.poses[0].position.z < 1.5)
        {
          close_enough_++;
          // TODO: Changing orientation. Calulate yaw from marker orientation

          if(close_enough_ > (SAFETY_TIME_SEC * ROS_RATE))
          {
            ROS_INFO("Close enough");
            break; // Exit loop and fly to final target
          }
        }
        else {close_enough_ = 0;}

        ros_client_->setpoint_pos_pub_.publish(endpoint_pos_ENU_);
        ros::spinOnce();

        approaching_ = true;
      }
    }
    else
    {
      approaching_ = false;
      cnt++;
      if(cnt % 66 == 0) ROS_WARN("No marker was found in the last 1 second");
    }
    ros::spinOnce();
    rate_->sleep();
  }
  if(j == MAX_ATTEMPTS) ROS_WARN("MAX_ATTEMPTS reached while approaching marker. Aborting.");

  // Publish final setpoint for 3 seconds before landing
  for(int i = 0; ros::ok() && i < 3 * ROS_RATE; ++i)
  {
    ros_client_->setpoint_pos_pub_.publish(endpoint_pos_ENU_);
    ros::spinOnce();
    rate_->sleep();
  }

  approaching_ = false;
  if(i < MAX_ATTEMPTS && j < MAX_ATTEMPTS) ROS_INFO("Marker approached!");

  return;
}

void DroneControl::land()
{
  int i;
  mavros_msgs::CommandTOL land_cmd;
  land_cmd.request.yaw = 0;
  land_cmd.request.latitude = NAN; //Land at current location
  land_cmd.request.longitude = NAN;
  land_cmd.request.altitude = 0;

  ROS_INFO("Trying to land");
  while(!(ros_client_->land_client_.call(land_cmd) && land_cmd.response.success))
  {
    ros_client_->setpoint_pos_pub_.publish(setpoint_pos_ENU_);
    ros::spinOnce();
    ROS_WARN("Retrying to land");
    rate_->sleep();
  }

  // Wait until proper landing (or a maximum of 15 seconds)
  for(i = 0; ros::ok() && landed_state_ != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND && i < MAX_ATTEMPTS; ++i)
  {
    ros::spinOnce();
    rate_->sleep();
  }
  if(i == MAX_ATTEMPTS)
    ROS_WARN("Landing failed, aborting");
  else
    ROS_INFO("Landing success");


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

double DroneControl::currentYaw()
{
  //Calculate yaw current orientation
  double roll, pitch, yaw;
  tf::Quaternion q;

  tf::quaternionMsgToTF(local_position_.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  return yaw;
}

double DroneControl::getYaw(const geometry_msgs::Quaternion &msg)
{
  //Calculate yaw current orientation
  double roll, pitch, yaw;
  tf::Quaternion q;

  tf::quaternionMsgToTF(msg, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  return yaw;
}

double DroneControl::distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2)
{
  tf::Point t1, t2;
  tf::pointMsgToTF(p1.pose.position, t1);
  tf::pointMsgToTF(p2.pose.position, t2);

  return t1.distance(t2);
}
