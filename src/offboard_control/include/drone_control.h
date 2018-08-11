/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include "ros_client.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

class ROSClient; // Forward declaration because of circular reference

class DroneControl
{
  public:
    DroneControl(ROSClient *ros_client);

    static constexpr float FLIGHT_ALTITUDE = 0.5;
    static constexpr float ROS_RATE = 20.0;
    static constexpr int   MAX_ATTEMPTS = 300;
    static constexpr int   SAFETY_TIME_SEC = 3;
    static constexpr float TURN_STEP_RAD = 4/ROS_RATE;
    static constexpr float INIT_FLIGHT_DURATION = 4.0; //In seconds per side
    static constexpr float INIT_FLIGHT_LENGTH = 1.0;   //In meters
    static constexpr int   INIT_FLIGHT_REPEAT = 5;     //Times
    static constexpr float TEST_FLIGHT_DURATION = 3.0; //In seconds per side
    static constexpr float TEST_FLIGHT_LENGTH = 4.0;   //In meters
    static constexpr int   TEST_FLIGHT_REPEAT = 2;     //Times
    static constexpr bool  KEEP_ALIVE = false;

    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate *rate_;
    tf2_ros::Buffer tfBuffer_;

    mavros_msgs::State current_state_;
    geometry_msgs::PoseArray marker_position_;
    geometry_msgs::PoseStamped local_position_;
    geometry_msgs::PoseWithCovarianceStamped svo_position_;
    geometry_msgs::TransformStamped transformStamped_;

    void state_cb(const mavros_msgs::State::ConstPtr &msg);
    void marker_position_cb(const geometry_msgs::PoseArray::ConstPtr &msg);
    void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void setpoint_position_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void svo_position_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    void offboardMode();
    void takeOff();
    void testFlightHorizontal();
    void testFlightVertical();
    void initVIO();
    void vioOff();
    void vioOn();
    void collisionAvoidOff();
    void collisionAvoidOn();
    void turnTowardsMarker();
    void approachMarker();
    void hover(int seconds);
    void land();
    void disarm();

  private:
    bool approaching_ = false;
    bool endpoint_active_ = false;
    bool send_vision_estimate_ = true;
    bool svo_running_ = false;
    unsigned char close_enough_ = 0;

    geometry_msgs::PoseStamped setpoint_pos_ENU_;
    geometry_msgs::PoseStamped endpoint_pos_ENU_;
    geometry_msgs::PoseStamped vision_pos_ENU_;
    geometry_msgs::PoseStamped gps_init_pos_;
    geometry_msgs::PoseStamped svo_init_pos_;

    ros::Time last_request_;
    ros::Time last_svo_estimate_;

    mavros_msgs::CommandBool arm_cmd_;
    std_msgs::String svo_cmd_;
    std_msgs::String ewok_cmd_;

    ROSClient *ros_client_;

    float currentYaw();
};


#endif /* DRONE_CONTROL_H */
