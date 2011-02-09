#include "laser_height_estimation/laser_height_estimation.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LaserHeightEstimation");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  LaserHeightEstimation laser_height_estimation(nh, nh_private);
  ros::spin();
  return 0;
}
