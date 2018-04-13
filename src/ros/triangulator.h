#ifndef __TRIANGULATOR_H__
#define __TRIANGULATOR_H__

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseArray.h>
#include <whycon/PointArray.h>

namespace whycon {
  class Triangulator {
    public:
      Triangulator(ros::NodeHandle& n);
      ~Triangulator(void);

      void on_points(const whycon::PointArray::ConstPtr& points_left, const whycon::PointArray::ConstPtr& points_right,
                    const geometry_msgs::PoseArray::ConstPtr& poses_left, const geometry_msgs::PoseArray::ConstPtr& poses_right);

    private:
      ros::Publisher viz_pub;
      cv::Mat P_left, P_right;
      cv::Mat K_left, K_right;
      cv::Mat dist_coeffs_left, dist_coeffs_right;
      cv::Mat R_left, R_right;
      cv::Mat R;

      message_filters::Subscriber<whycon::PointArray> points_left_sub, points_right_sub;
      message_filters::Subscriber<geometry_msgs::PoseArray> poses_left_sub, poses_right_sub;
      message_filters::TimeSynchronizer<whycon::PointArray, whycon::PointArray, geometry_msgs::PoseArray, geometry_msgs::PoseArray>* sync;
  };
}

#endif
