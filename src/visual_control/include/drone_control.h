/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <math.h>

#define FLIGHT_ALTITUDE 0.5
#define ROS_RATE 20.0
#define MAX_ATTEMPTS 300
#define SAFETY_TIME_SEC 3
#define TURN_STEP_RAD 4/ROS_RATE
#define INIT_FLIGHT_DURATION 4.0 //In seconds per side
#define INIT_FLIGHT_LENGTH 1.0   //In meters
#define INIT_FLIGHT_REPEAT 5     //Times
#define TEST_FLIGHT_DURATION 3.0 //In seconds per side
#define TEST_FLIGHT_LENGTH 4.0   //In meters
#define TEST_FLIGHT_REPEAT 2     //Times
#define KEEP_ALIVE false

void offboardMode();
void takeOff();
void testFlightHorizontal();
void testFlightVertical();
void initVIO();
void turnTowardsMarker();
void approachMarker();
void land();
void disarm();
float currentYaw();

extern bool approaching;
extern bool send_vision_estimate;
extern unsigned char close_enough;
extern geometry_msgs::PoseStamped setpoint_pos_ENU;
extern geometry_msgs::PoseStamped vision_pos_ENU;
extern geometry_msgs::PoseStamped svo_init_pos;
extern ros::Time last_request;
extern ros::Time last_svo_estimate;
extern mavros_msgs::CommandBool arm_cmd;
extern tf2_ros::Buffer tfBuffer;
extern std_msgs::String svo_cmd;

extern mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg);
extern geometry_msgs::PoseArray marker_position;
void marker_position_cb(const geometry_msgs::PoseArray::ConstPtr& msg);
extern geometry_msgs::PoseStamped local_position;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
extern geometry_msgs::PoseWithCovarianceStamped svo_position;
void svo_position_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);


#endif /* DRONE_CONTROL_H */
