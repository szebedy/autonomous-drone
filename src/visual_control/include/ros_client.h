#ifndef ROS_CLIENT_H
#define ROS_CLIENT_H

#include <ros/ros.h>

class DroneControl; // Forward declaration because of circular reference

class ROSClient
{
  public:
    ROSClient();

    void init(DroneControl *drone_control);

    ros::Subscriber state_sub_;
    ros::Subscriber marker_pos_sub_;
    ros::Subscriber local_pos_sub_;
    ros::Subscriber svo_pos_sub_;

    ros::Publisher setpoint_pos_pub_;
    ros::Publisher vision_pos_pub_;
    ros::Publisher svo_cmd_pub_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient land_client_;
    ros::ServiceClient set_mode_client_;
};

#endif /* ROS_CLIENT_H */
