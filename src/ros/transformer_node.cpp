#include <ros/ros.h>
#include "transformer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transformer");
  ros::NodeHandle n("~");

  whycon::Transformer t(n);
  ros::spin();
}
