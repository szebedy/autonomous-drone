/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include "drone_control.h"

ros::Subscriber state_sub;
ros::Subscriber marker_pos_sub;
ros::Subscriber local_pos_sub;
ros::Subscriber svo_pos_sub;

ros::Publisher setpoint_pos_pub;
ros::Publisher vision_pos_pub;
ros::Publisher svo_cmd_pub;

ros::ServiceClient arming_client;
ros::ServiceClient land_client;
ros::ServiceClient set_mode_client;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");

    ros::NodeHandle nh;

    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(ROS_RATE);

    static tf2_ros::TransformListener tfListener(tfBuffer);

    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    marker_pos_sub = nh.subscribe<geometry_msgs::PoseArray>("/whycon/poses", 10, marker_position_cb);
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_position_cb);
    svo_pos_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/svo/pose_imu", 10, svo_position_cb);

    setpoint_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    vision_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    svo_cmd_pub = nh.advertise<std_msgs::String>("/svo/remote_key", 10);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // Wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCT...");
    }

    last_svo_estimate = ros::Time::now();

    offboardMode();

    // The tf module of mavros does not work currently. See also approachMarker function
    //nh.setParam("/mavros/local_position/tf/frame_id", "map");
    //nh.setParam("/mavros/local_position/tf/child_frame_id", "drone");
    //nh.setParam("/mavros/local_position/tf/send", true);

    takeOff();

    initVIO();

    //testFlightHorizontal();
    testFlightVertical();

    //turnTowardsMarker();

    //approachMarker(nh);

    land();
    disarm();

    while(ros::ok() && KEEP_ALIVE) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
