
#include "transformer.h"

whycon::Transformer::Transformer(ros::NodeHandle &n)
{
  has_projection = false;

  n.param("world_frame", world_frame_id, std::string("world"));

  poses_pub = n.advertise<geometry_msgs::PoseArray>("poses", 1);
  poses_2d_pub = n.advertise<geometry_msgs::PoseArray>("poses_2d", 1);

  poses_sub.subscribe(n, "/whycon/poses", 1);
  projection_sub.subscribe(n, "/whycon/projection", 1);
  synchronizer = boost::make_shared<message_filters::TimeSynchronizer<geometry_msgs::PoseArray, whycon::Projection>>(poses_sub, projection_sub, 10);
  synchronizer->registerCallback(boost::bind(&Transformer::on_poses, this, _1, _2));

  transform_listener = boost::make_shared<tf::TransformListener>();
}

void whycon::Transformer::on_poses(const geometry_msgs::PoseArrayConstPtr& poses_msg, const whycon::ProjectionConstPtr& projection_msg)
{
  if (poses_pub.getNumSubscribers() > 0)
  {
    if (!transform_listener->canTransform(world_frame_id, poses_msg->header.frame_id, ros::Time(0)))
      return;

    tf::StampedTransform t;
    transform_listener->lookupTransform(world_frame_id, poses_msg->header.frame_id, ros::Time(0), t);

    geometry_msgs::PoseArray transformed_poses;
    transformed_poses.header = poses_msg->header;
    transformed_poses.header.frame_id = world_frame_id;

    transformed_poses.poses.resize(poses_msg->poses.size());
    for (size_t i = 0; i < poses_msg->poses.size(); i++) {
      tf::Point point;
      tf::pointMsgToTF(poses_msg->poses[i].position, point);
      tf::Point transformed_point = t * point;
      tf::pointTFToMsg(transformed_point, transformed_poses.poses[i].position);
    }

    poses_pub.publish(transformed_poses);
  }

  if (poses_2d_pub.getNumSubscribers() > 0)
  {
    if (!has_projection)
    {
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
          projection(i,j) = projection_msg->projection[3 * i + j];

      has_projection = true;
    }

    geometry_msgs::PoseArray transformed_poses;
    transformed_poses.header = poses_msg->header;
    transformed_poses.header.frame_id = world_frame_id;

    transformed_poses.poses.resize(poses_msg->poses.size());
    for (size_t i = 0; i < poses_msg->poses.size(); i++) {
      tf::Point point;
      tf::pointMsgToTF(poses_msg->poses[i].position, point);

      cv::Vec3d vec(point.getX(), point.getY(), point.getZ());
      vec = projection * vec;

      tf::pointTFToMsg(tf::Point(vec(0) / vec(2), vec(1) / vec(2), 0), transformed_poses.poses[i].position);
    }

    poses_2d_pub.publish(transformed_poses);
  }
}

