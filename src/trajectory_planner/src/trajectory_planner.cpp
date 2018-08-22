/**
* This file is part of Ewok.
*
* Copyright 2017 Vladyslav Usenko, Technical University of Munich.
* Developed by Vladyslav Usenko <vlad dot usenko at tum dot de>,
* for more information see <http://vision.in.tum.de/research/robotvision/replanning>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Ewok is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Ewok is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Ewok. If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <ewok/ed_ring_buffer.h>
#include <sensor_msgs/Image.h>
#include <cholmod.h>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/String.h>

#include <ewok/polynomial_3d_optimization.h>
#include <ewok/uniform_bspline_3d_optimization.h>


static const uint8_t POW = 6;
static const uint8_t subsample = 4;
static const double dt = 0.5;
static const int num_opt_points = 7;
static const double max_velocity = 1.0;
static const double max_acceleration = 0.5;
static const double resolution = 0.1;
static const double distance_threshold = 0.3;

bool ringbufferActive = false;
bool ringbufferInitialized = false;
bool setpointActive = false;
bool setpointInitialized = false;
bool encodingInitialized = false;
bool encodingFloat = false;

ros::Subscriber local_pos_sub;
ros::Subscriber endpoint_pos_sub;
ros::Subscriber depth_cam_sub;
ros::Subscriber ewok_cmd_sub;

ros::Publisher setpoint_pos_pub;
ros::Publisher occ_marker_pub, free_marker_pub, dist_marker_pub, current_traj_marker_pub, traj_marker_pub;

geometry_msgs::PoseStamped endpoint_position;
geometry_msgs::PoseStamped local_position;
geometry_msgs::PoseStamped setpoint_pos_ENU;
sensor_msgs::Image depth_cam_img;

ewok::PolynomialTrajectory3D<10>::Ptr traj;
ewok::EuclideanDistanceRingBuffer<POW>::Ptr edrb;
ewok::UniformBSpline3DOptimization<6>::Ptr spline_optimization;

tf::TransformListener * listener;

void ewok_cmd_cb(const std_msgs::String::ConstPtr& msg)
{
  if(msg->data == "s")
  {
    ringbufferActive = true;
  }
  if(msg->data == "r")
  {
    ringbufferActive = false;
    setpointActive = false;
  }
}

void endpoint_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  endpoint_position = *msg;

  ringbufferActive = true;

  // Set up desired trajectory
  Eigen::Vector3d start_point(local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z),
                  end_point(endpoint_position.pose.position.x, endpoint_position.pose.position.y, endpoint_position.pose.position.z);

  ROS_INFO("Requested trajectory start: %f %f %f, stop: %f %f %f",
           local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z,
           endpoint_position.pose.position.x, endpoint_position.pose.position.y, endpoint_position.pose.position.z);

  Eigen::Vector4d limits(max_velocity, max_acceleration, 0, 0);

  ewok::Polynomial3DOptimization<10> to(limits);
  typename ewok::Polynomial3DOptimization<10>::Vector3Array path;
  path.push_back(start_point);
  path.push_back(end_point);

  traj = to.computeTrajectory(path);

  visualization_msgs::MarkerArray traj_marker;
  traj->getVisualizationMarkerArray(traj_marker, "gt", Eigen::Vector3d(1,0,1));
  traj_marker_pub.publish(traj_marker);

  spline_optimization.reset(new ewok::UniformBSpline3DOptimization<6>(traj, dt));

  for (int i = 0; i < num_opt_points; i++) {
      spline_optimization->addControlPoint(start_point);
  }

  spline_optimization->setNumControlPointsOptimized(num_opt_points);
  spline_optimization->setDistanceBuffer(edrb);
  spline_optimization->setDistanceThreshold(distance_threshold);
  spline_optimization->setLimits(limits);

  setpointActive = true;
}

void depth_cam_cb(const sensor_msgs::Image::ConstPtr& msg)
{
  if(!encodingInitialized)
  {
    if(msg->encoding == "32FC1")
    {
      encodingFloat = true;
      ROS_INFO("Depth image encoding: 32FC1");
    }
    else if(msg->encoding == "16UC1")
    {
      encodingFloat = false;
      ROS_INFO("Depth image encoding: 16UC1");
    }
    else
    {
      ROS_INFO("Couldn't get encoding");
    }
    encodingInitialized = true;
  }
  if(ringbufferActive)
  {
    static const float fx = 457.815979003906;
    static const float fy = 457.815979003906;
    static const float cx = 249.322647094727;
    static const float cy = 179.5;

    tf::StampedTransform transform;

    try
    {
      listener->lookupTransform("world", "camera", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_INFO("Couldn't get transform");
      ROS_WARN("%s",ex.what());
      return;
    }

    Eigen::Affine3d dT_w_c;
    tf::transformTFToEigen(transform, dT_w_c);

    Eigen::Affine3f T_w_c = dT_w_c.cast<float>();

    //auto t1 = std::chrono::high_resolution_clock::now();

    ewok::EuclideanDistanceRingBuffer<POW>::PointCloud cloud1;

    for(int v=0; v < msg->height; v+=subsample)
    {
      for(int u=0; u < msg->width; u+=subsample)
      {
        float val;
        if(encodingFloat)
        {
          val = *(float *)&msg->data[(v*msg->width + u)*4];
        }
        else
        {
          uint16_t uval = *(uint16_t *)&msg->data[(v*msg->width + u)*2];
          val = uval/1000.0; //Depth data is represented in mm
        }

        if(std::isfinite(val) && val > 0.05)
        {
          Eigen::Vector4f p;
          p[0] = val*(u - cx)/fx;
          p[1] = val*(v - cy)/fy;
          p[2] = val;
          p[3] = 1;

          p = T_w_c * p;

          cloud1.push_back(p);
        }
      }
    }

    Eigen::Vector3f origin = (T_w_c * Eigen::Vector4f(0,0,0,1)).head<3>();

    //auto t2 = std::chrono::high_resolution_clock::now();

    if(!ringbufferInitialized)
    {
      Eigen::Vector3i idx;
      edrb->getIdx(origin, idx);

      ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());

      edrb->setOffset(idx);

      ringbufferInitialized = true;
    }
    else
    {
      Eigen::Vector3i origin_idx, offset, diff;
      edrb->getIdx(origin, origin_idx);

      offset = edrb->getVolumeCenter();
      diff = origin_idx - offset;

      while(diff.array().any())
      {
        //ROS_INFO("Moving Volume");
        edrb->moveVolume(diff);

        offset = edrb->getVolumeCenter();
        diff = origin_idx - offset;
      }
    }

    //ROS_INFO_STREAM("cloud1 size: " << cloud1.size());

    //auto t3 = std::chrono::high_resolution_clock::now();

    edrb->insertPointCloud(cloud1, origin);

    //auto t4 = std::chrono::high_resolution_clock::now();

    //f_time << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << " " <<
    //          std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << " " <<
    //          std::chrono::duration_cast<std::chrono::nanoseconds>(t4-t3).count() << std::endl;

    visualization_msgs::Marker m_occ, m_free;
    edrb->getMarkerOccupied(m_occ);
    edrb->getMarkerFree(m_free);

    occ_marker_pub.publish(m_occ);
    free_marker_pub.publish(m_free);
  }
}

void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  local_position = *msg;

  if(!setpointInitialized)
  {
    setpoint_pos_ENU = local_position;
    setpointInitialized = true;
  }
}

void publishSetpoint(const ros::TimerEvent& e)
{
  if(setpointActive)
  {
    setpoint_pos_ENU.pose.orientation = endpoint_position.pose.orientation;
    //ROS_INFO("Publish: %f %f %f", setpoint_pos_ENU.pose.position.x, setpoint_pos_ENU.pose.position.y, setpoint_pos_ENU.pose.position.z);
    setpoint_pos_pub.publish(setpoint_pos_ENU);

    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_avoid");
  ros::NodeHandle nh;

  local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_position_cb);
  endpoint_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/trajectory/endpoint_position", 10, endpoint_position_cb);
  ewok_cmd_sub = nh.subscribe<std_msgs::String>("/trajectory/command", 10, ewok_cmd_cb);

  listener = new tf::TransformListener;

  occ_marker_pub = nh.advertise<visualization_msgs::Marker>("/ring_buffer/occupied", 5);
  free_marker_pub = nh.advertise<visualization_msgs::Marker>("/ring_buffer/free", 5);
  dist_marker_pub = nh.advertise<visualization_msgs::Marker>("/ring_buffer/distance", 5);
  traj_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/global_trajectory", 1, true);
  current_traj_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/optimal_trajectory", 1, true);

  while(!ringbufferActive)
  {
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }

  depth_cam_sub = nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 1, depth_cam_cb);

  setpoint_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/trajectory/setpoint_position", 10);

  edrb.reset(new ewok::EuclideanDistanceRingBuffer<POW>(resolution, 1.0));

  // The setpoint publishing rate MUST be faster than 2Hz
  ros::Timer timer = nh.createTimer(ros::Duration(0.05), publishSetpoint);

  // Trigger callback functions at the following rate
  ros::Rate r(dt);
  while(ros::ok())
  {
    ros::spinOnce();

    //auto t1 = std::chrono::high_resolution_clock::now();

    edrb->updateDistance();

    visualization_msgs::MarkerArray traj_marker;

    //auto t2 = std::chrono::high_resolution_clock::now();
    if(setpointActive)
    {
      spline_optimization->optimize();
      //auto t3 = std::chrono::high_resolution_clock::now();

      //opt_time << std::chrono::duration_cast<std::chrono::nanoseconds>(t2-t1).count() << " "
      //    << std::chrono::duration_cast<std::chrono::nanoseconds>(t3-t2).count() << std::endl;

      Eigen::Vector3d pc = spline_optimization->getFirstOptimizationPoint();

      setpoint_pos_ENU.pose.position.x = pc[0];
      setpoint_pos_ENU.pose.position.y = pc[1];
      setpoint_pos_ENU.pose.position.z = pc[2];

      spline_optimization->getMarkers(traj_marker);
      current_traj_marker_pub.publish(traj_marker);

      spline_optimization->addLastControlPoint();
\
      visualization_msgs::Marker m_dist;
      edrb->getMarkerDistance(m_dist, 0.5);
      dist_marker_pub.publish(m_dist);
    }

    r.sleep();
  }

  return 0;
}
