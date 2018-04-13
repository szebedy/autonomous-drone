#include "robot_pose_publisher.h"
#include <geometry_msgs/Pose.h>
#include <angles/angles.h>

whycon::RobotPosePublisher::RobotPosePublisher(ros::NodeHandle& n)
{
  n.param("axis_length_tolerance", axis_length_tolerance, 0.05);
  n.param("world_frame", world_frame, std::string("world"));
  n.param("target_frame", target_frame, std::string("target"));

  broadcaster = boost::make_shared<tf::TransformBroadcaster>();

  pose_sub = n.subscribe<geometry_msgs::PoseArray>("/whycon/trans_poses", 1, &whycon::RobotPosePublisher::on_poses, this);
}

/* this assumes an L-shaped pattern, defining the two axis of the robot on the plane (forward and left are positive) */
void whycon::RobotPosePublisher::on_poses(const geometry_msgs::PoseArrayConstPtr& pose_array)
{
  ROS_INFO_STREAM("receiving poses");
  tf::Transform T;

  if (pose_array->poses.size() != 3) { ROS_WARN_STREAM("More/less than three circles detected, will not compute pose"); return; }

  const std::vector<geometry_msgs::Pose>& ps = pose_array->poses;
  float dists[3];
  ROS_INFO_STREAM("poses: " << ps[0].position << " " << ps[1].position << " " << ps[2].position);

  dists[0] = sqrt(pow(ps[0].position.x - ps[1].position.x, 2) + pow(ps[0].position.y - ps[1].position.y, 2)); // d(0,1)
  dists[1] = sqrt(pow(ps[1].position.x - ps[2].position.x, 2) + pow(ps[1].position.y - ps[2].position.y, 2)); // d(1,2)
  dists[2] = sqrt(pow(ps[2].position.x - ps[0].position.x, 2) + pow(ps[2].position.y - ps[0].position.y, 2)); // d(2,0)
  ROS_INFO_STREAM("distances: " << dists[0] << " " << dists[1] << " " << dists[2]);
  size_t max_idx = (std::max_element(dists, dists + 3) - dists);
  std::swap(dists[max_idx], dists[0]); // dists[0] is now the longer distance (hypothenuse)


  /* the two smaller distances correspond to the two cathetus, which should be equal in length (up to a certain tolerance) */
  if (fabsf(dists[1] - dists[2]) > axis_length_tolerance) { ROS_WARN_STREAM("Axis size differ: " << dists[1] << " , " << dists[2]); return ; }
  if (fabsf(dists[0] - sqrt(pow(dists[1],2) + pow(dists[2],2))) > axis_length_tolerance) { ROS_WARN_STREAM("Pythagoras check not passed"); return ; }

  /* compute robot position as center of coordinate frame */
  const geometry_msgs::Pose& center = ps[(max_idx + 2) % 3];
  T.setOrigin(tf::Vector3(center.position.x, center.position.y, 0.0));

  /* compute robot orientation as orientation of coordinate frame */
  const geometry_msgs::Pose& p1 = ps[(max_idx + 0) % 3];
  const geometry_msgs::Pose& p2 = ps[(max_idx + 1) % 3];

  float v1[2], v2[2];
  v1[0] = p1.position.x - center.position.x; v1[1] = p1.position.y - center.position.y;
  v2[0] = p2.position.x - center.position.x; v2[1] = p2.position.y - center.position.y;

  float n1 = sqrt(v1[0] * v1[0] + v1[1] * v1[1]);
  float n2 = sqrt(v2[0] * v2[0] + v2[1] * v2[1]);

  v1[0] /= n1; v1[1] /= n1;
  v2[0] /= n2; v2[1] /= n2;

  if (angles::normalize_angle(atan2(v1[1], v1[0]) - atan2(v2[1], v2[0])) > 0)
    T.setRotation(tf::createQuaternionFromYaw(atan2(v2[1], v2[0])));
  else
    T.setRotation(tf::createQuaternionFromYaw(atan2(v1[1], v1[0])));

  broadcaster->sendTransform(tf::StampedTransform(T, pose_array->header.stamp, world_frame, target_frame));
}

