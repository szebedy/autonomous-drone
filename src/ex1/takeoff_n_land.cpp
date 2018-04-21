/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf/tf.h>

#define FLIGHT_ALTITUDE 1.5f

ros::Subscriber state_sub;
ros::Subscriber pose_sub;

ros::Publisher local_pos_pub;
ros::Publisher setpoint_pub;

ros::ServiceClient arming_client;
ros::ServiceClient land_client;
ros::ServiceClient set_mode_client;

geometry_msgs::PoseStamped pose;

void offboardMode();
void takeOff();
void approachMarker();
void land();

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseArray current_pose;
void pose_cb(const geometry_msgs::PoseArray::ConstPtr& msg){
    current_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");

    ros::NodeHandle nh;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    pose_sub = nh.subscribe<geometry_msgs::PoseArray>("whycon/poses", 10, pose_cb);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCT...");
    }

    mavros_msgs::PositionTarget setpoint;

    offboardMode();

    takeOff();

    approachMarker();

    land();

    return 0;
}

void offboardMode(){
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    //send a few setpoints before starting, otherwise px4 will not switch to OFFBOARD mode
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // change to offboard mode and arm
    while(ros::ok() && !current_state.armed){
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            ROS_INFO(current_state.mode.c_str());
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    return;
}

void takeOff(){
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // Take off
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.5f);

    ROS_INFO("Taking off");
    for(int i = 0; ros::ok() && i < 10*20; ++i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Takeoff finished! Looking for whycon marker");
    return;
}

void approachMarker(){
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    /*for(int j = 0; ros::ok() && j < 10; ++j){
      if (ros::Time::now() - current_pose.header.stamp < ros::Duration(5.0)) {
        ROS_INFO("Marker found, approaching");
        if (current_pose.poses[0].position.z < 2) {
          ROS_INFO("Changing orientation");
          pose.pose.orientation = current_pose.poses[0].orientation;
        }
        if (current_pose.poses[0].position.z < 1) {
          if (current_pose.poses[0].position.z < 0.6) {
            ROS_INFO("Close enough");
            break;
          }
          ROS_INFO("Close enough, last move");
          j = 10;
        }
        // go towards the marker
        pose.pose.position.x += current_pose.poses[0].position.z/2;
        pose.pose.position.y -= current_pose.poses[0].position.x;
        pose.pose.position.z -= current_pose.poses[0].position.y;
        //send setpoint for 5 seconds
        for(int i = 0; ros::ok() && i < 5*20; ++i){
          setpoint_pub.publish(pose);
          ros::spinOnce();
          rate.sleep();
        }
      }
      else
        ROS_INFO("No marker was found in the last 5 seconds");
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Marker approached!");

    // Back to the origin
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    for(int i = 0; ros::ok() && i < 10*20; ++i){
      setpoint_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }*/
}

void land(){
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    ROS_INFO("Tring to land");
    while (!(land_client.call(land_cmd) && land_cmd.response.success)){
        //local_pos_pub.publish(pose);
        ROS_INFO("Tring to land");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Landed");
    return;
}
