#ifndef COLLISION_AVOID_H
#define COLLISION_AVOID_H

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>

class CollisionAvoid
{
  public:
    CollisionAvoid();

    bool avoid_ = false;

    geometry_msgs::PoseStamped avoid(geometry_msgs::PoseStamped setpoint_pos_ENU, const sensor_msgs::Image &depth_cam_img);
  private:

};

#endif /* COLLISION_AVOID_H */
