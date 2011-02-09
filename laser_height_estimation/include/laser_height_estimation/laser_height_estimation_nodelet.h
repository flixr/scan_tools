#ifndef LASER_HEIGHT_ESTIMATION_LASER_HEIGHT_ESTIMATION_NODELET_H
#define LASER_HEIGHT_ESTIMATION_LASER_HEIGHT_ESTIMATION_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "laser_height_estimation/laser_height_estimation.h"

class LaserHeightEstimationNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit ();

  private:
    LaserHeightEstimation * laser_height_estimation_;  // FIXME: change to smart pointer
};


#endif // LASER_HEIGHT_ESTIMATION_LASER_HEIGHT_ESTIMATION_NODELET_H
