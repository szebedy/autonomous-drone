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
#include <tf/transform_datatypes.h>
#include <tf2/buffer_core.h>
#include <math.h>

#define FLIGHT_ALTITUDE 1.0f

#define ROS_RATE 20.0

ros::Subscriber state_sub;
ros::Subscriber pose_sub;
ros::Subscriber local_pos_sub;

ros::Publisher local_pos_pub;
ros::Publisher setpoint_pub;

ros::ServiceClient arming_client;
ros::ServiceClient land_client;
ros::ServiceClient set_mode_client;

void offboardMode();
void takeOff();
void turnTowardsMarker();
void approachMarker();
void land();
float currentYaw();

geometry_msgs::PoseStamped pose_NED;
ros::Time last_request;
mavros_msgs::CommandBool arm_cmd;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseArray marker_pose;
void pose_cb(const geometry_msgs::PoseArray::ConstPtr& msg){
    marker_pose = *msg;
}

geometry_msgs::PoseStamped local_position;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");

    ros::NodeHandle nh;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(ROS_RATE);

    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    pose_sub = nh.subscribe<geometry_msgs::PoseArray>("whycon/poses", 10, pose_cb);
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_position_cb);

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

    offboardMode();

    takeOff();

    turnTowardsMarker();

    //ROS_INFO("%f, %f, %f, %f, %f, %f, %f", local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z,
    //          local_position.pose.orientation.x, local_position.pose.orientation.y, local_position.pose.orientation.z, local_position.pose.orientation.w);

    //approachMarker();

    land();

    return 0;
}

void offboardMode(){
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(ROS_RATE);

    pose_NED.pose.position.x = local_position.pose.position.x;
    pose_NED.pose.position.y = local_position.pose.position.y;
    pose_NED.pose.position.z = FLIGHT_ALTITUDE;

    //send a few setpoints before starting, otherwise px4 will not switch to OFFBOARD mode
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose_NED);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    arm_cmd.request.value = true;

    last_request = ros::Time::now();

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
        local_pos_pub.publish(pose_NED);
        ros::spinOnce();
        rate.sleep();
    }
    return;
}

void takeOff(){
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(ROS_RATE);

    // Take off
    pose_NED.pose.position.x = local_position.pose.position.x;
    pose_NED.pose.position.y = local_position.pose.position.y;
    pose_NED.pose.position.z = FLIGHT_ALTITUDE;
    //pose_NED.pose.orientation = tf::createQuaternionMsgFromYaw(-0.7f);

    ROS_INFO("Taking off");
    for(int i = 0; ros::ok() && i < 10 * ROS_RATE; ++i){
        local_pos_pub.publish(pose_NED);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Takeoff finished! Looking for whycon marker");
    return;
}

void turnTowardsMarker(){
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(ROS_RATE);
    float rad, current_yaw;

    for(int j = 0; ros::ok() && j < 5 * ROS_RATE; ++j){
        if (ros::Time::now() - marker_pose.header.stamp < ros::Duration(1.0)) {
            //Calculate yaw angle difference of marker in radians
            rad = -atan2f(marker_pose.poses[0].position.x, marker_pose.poses[0].position.z);
            if (fabs(rad) < 0.03) {
                ROS_INFO("Headed towards marker!");
                break;
            }

            current_yaw = currentYaw();

            ROS_INFO("Marker found, current yaw: %f, turning %f radians", current_yaw, rad);
            // turn towards the marker without change of position
            pose_NED.pose.position.x = local_position.pose.position.x;
            pose_NED.pose.position.y = local_position.pose.position.y;
            pose_NED.pose.position.z = local_position.pose.position.z;
            pose_NED.pose.orientation = tf::createQuaternionMsgFromYaw(current_yaw+rad);
            //send setpoint for 5 seconds
            for(int i = 0; ros::ok() && i < 5 * ROS_RATE; ++i){
                local_pos_pub.publish(pose_NED);
                ros::spinOnce();
                rate.sleep();
            }
        } else {
            ROS_INFO("No marker was found in the last second");
            ros::spinOnce();
            rate.sleep();
        }
    }
    return;
}

void approachMarker(){
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(ROS_RATE);

    tf2::BufferCore buffer_core;

    //Transformation from map to drone coordinates
    geometry_msgs::TransformStamped tf_map2drone;
    tf_map2drone.header.frame_id = "map";
    tf_map2drone.child_frame_id = "drone";

    //Transformation from drone to visual marker coordinates
    geometry_msgs::TransformStamped tf_drone2marker;
    tf_drone2marker.header.frame_id = "drone";
    tf_drone2marker.child_frame_id = "marker";

    //Transformation from map to visual target coordinates
    geometry_msgs::TransformStamped ts_lookup;

    for(int j = 0; ros::ok() && j < 10; ++j){
      if (ros::Time::now() - marker_pose.header.stamp < ros::Duration(5.0)) {
        ROS_INFO("Marker found, approaching");
        if (marker_pose.poses[0].position.z < 2) {
          ROS_INFO("TODO: Changing orientation");
          //pose.pose.orientation = marker_pose.poses[0].orientation;
        }
        if (marker_pose.poses[0].position.z < 1) {
          if (marker_pose.poses[0].position.z < 0.6) {
            ROS_INFO("Close enough");
            break;
          }
          ROS_INFO("Close enough, last move");
          j = 10;
        }

        //Transformation from map to drone coordinates
        tf_map2drone.transform.translation.x = local_position.pose.position.x;
        tf_map2drone.transform.translation.y = local_position.pose.position.y;
        tf_map2drone.transform.translation.z = local_position.pose.position.z;
        tf_map2drone.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, currentYaw());
        buffer_core.setTransform(tf_map2drone, "default_authority");

        //Transformation from drone to visual marker coordinates
        tf_drone2marker.transform.translation.x = marker_pose.poses[0].position.z/2; //Only fly half of the forward distance
        tf_drone2marker.transform.translation.y = -marker_pose.poses[0].position.x;
        tf_drone2marker.transform.translation.z = -marker_pose.poses[0].position.y;
        tf_drone2marker.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        buffer_core.setTransform(tf_drone2marker, "default_authority");

        //Look up transformation from map to visual target coordinates
        ts_lookup = buffer_core.lookupTransform("map", "marker", ros::Time(0));

        // go towards the marker
        pose_NED.pose.position.x = ts_lookup.transform.translation.x;
        pose_NED.pose.position.y = ts_lookup.transform.translation.y;
        pose_NED.pose.position.z = ts_lookup.transform.translation.z;
        //pose_NED.pose.orientation = tf::createQuaternionMsgFromYaw(currentYaw());
        //send setpoint for 5 seconds
        for(int i = 0; ros::ok() && i < 5 * ROS_RATE; ++i){
          local_pos_pub.publish(pose_NED);
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

    /*mavros_msgs::PositionTarget setpoint;

    setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED;
    setpoint.header.frame_id = "drone";
    setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    setpoint.header.stamp = ros::Time::now();

    setpoint.position.x = 0.0f; //N
    setpoint.position.y = 2.0f; //W
    setpoint.position.z = 2.0f; //U

    setpoint.velocity.x = 0.0f;
    setpoint.velocity.y = 0.0f;
    setpoint.velocity.z = 0.0f;

    setpoint.acceleration_or_force.x = 0.0f;
    setpoint.acceleration_or_force.y = 0.0f;
    setpoint.acceleration_or_force.z = 0.0f;

    setpoint.yaw = 0.0f;
    setpoint.yaw_rate = 0.0f;

    ROS_INFO("Publishing FRAME_BODY_NED position");
    for(int i = 0; ros::ok() && i < 10 * ROS_RATE; ++i){
        setpoint_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Done waiting");*/
    return;
}

void land(){
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(ROS_RATE);

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    ROS_INFO("Trying to land");
    while (!(land_client.call(land_cmd) && land_cmd.response.success)){
        //local_pos_pub.publish(pose);
        ROS_INFO("Retrying to land");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Success");

    //Wait 5 seconds for proper landing
    for(int i = 0; ros::ok() && i < 5 * ROS_RATE; ++i){
      ros::spinOnce();
      rate.sleep();
    }

    arm_cmd.request.value = false;
    // disarm
    while(ros::ok() && current_state.armed){
        if( current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                ROS_INFO("Vehicle disarmed");
            }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }
    return;
}


float currentYaw(){
    //Calculate yaw current orientation
    double roll, pitch, yaw;
    tf::Quaternion q(local_position.pose.orientation.x,
                     local_position.pose.orientation.y,
                     local_position.pose.orientation.z,
                     local_position.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    return yaw;
}
