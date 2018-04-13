#ifndef ROBOT_POSE_PUBLISHER_H
#define ROBOT_POSE_PUBLISHER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace whycon {
  class RobotPosePublisher {
    public:
      RobotPosePublisher(ros::NodeHandle& n);

      ros::Subscriber pose_sub;
      boost::shared_ptr<tf::TransformBroadcaster> broadcaster;

      double axis_length_tolerance;
      std::string world_frame, target_frame, axis_file;
      void on_poses(const geometry_msgs::PoseArrayConstPtr& pose_array);
  };
}

#endif // ROBOT_POSE_PUBLISHER_H
