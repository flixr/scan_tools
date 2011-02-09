#include "laser_height_estimation/laser_height_estimation_nodelet.h"

PLUGINLIB_DECLARE_CLASS (
  laser_height_estimation, LaserHeightEstimationNodelet, 
  LaserHeightEstimationNodelet, nodelet::Nodelet);

void LaserHeightEstimationNodelet::onInit ()
{
  NODELET_INFO("Initializing LaserHeightEstimation Nodelet");
  
  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  laser_height_estimation_ = new LaserHeightEstimation(nh, nh_private);  
}
