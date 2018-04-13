#include <ros/ros.h>
#include "triangulator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "triangulator");
  ros::NodeHandle n("~");

  whycon::Triangulator t(n);
  ros::spin();
}
