#include <ros/ros.h>
#include "set_axis.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "set_axis");
  ros::NodeHandle n("~");

  whycon::AxisSetter axis_setter(n);
  ros::spin();
}

