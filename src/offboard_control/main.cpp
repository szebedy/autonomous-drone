/**
 * @file main.cpp
 * @brief offboard control node, written with mavros version 0.26.0,
 * px4 flight stack 1.8.0 and tested on Intel Aero RTF and in Gazebo SITL
 */

#include "include/drone_control.h"
#include "include/ros_client.h"

int main(int argc, char **argv)
{
  ROSClient ros_client(argc, argv);
  DroneControl drone_control(&ros_client);

  drone_control.offboardMode();

  drone_control.vioOff();

  drone_control.takeOff();

  //drone_control.flyToGlobal(47.397728, 8.546135, 550, 0); //Above simulated apartment
  drone_control.flyToLocal(38, -1, DroneControl::SAFETY_ALTITUDE_GPS, 1.5708);
  drone_control.initVIO();

  drone_control.collisionAvoidOn();

  //drone_control.testFlightHorizontal();
  //drone_control.testFlightVertical();

  //drone_control.hover(10);

  drone_control.turnTowardsMarker();

  drone_control.approachMarker();

  drone_control.land();
  drone_control.disarm();

  while(ros::ok() && DroneControl::KEEP_ALIVE)
  {
    ros::spinOnce();
    drone_control.rate_->sleep();
  }

  return 0;
}
