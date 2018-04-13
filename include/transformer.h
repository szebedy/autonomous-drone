#ifndef WHYCON_TRANSFORMER_H
#define WHYCON_TRANSFORMER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <whycon/Projection.h>
#include <opencv2/opencv.hpp>

namespace whycon
{
  class Transformer {
    public:
      Transformer(ros::NodeHandle& n);

      ros::Publisher poses_pub, poses_2d_pub;

      message_filters::Subscriber<geometry_msgs::PoseArray> poses_sub;
      message_filters::Subscriber<whycon::Projection> projection_sub;
      boost::shared_ptr<message_filters::TimeSynchronizer<geometry_msgs::PoseArray, whycon::Projection>> synchronizer;

      std::string world_frame_id;

      cv::Matx33d projection;
      bool has_projection;

      void on_poses(const geometry_msgs::PoseArrayConstPtr& poses_msg, const ProjectionConstPtr& projection_msg);

      boost::shared_ptr<tf::TransformListener> transform_listener;
  };
}

#endif
