/*
*  Laser Ortho Projector
*  Copyright (C) 2010, CCNY Robotics Lab
*  Ivan Dryanovski <ivan.dryanovski@gmail.com>
*  William Morris <morris@ee.ccny.cuny.edu>
*  http://robotics.ccny.cuny.edu
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "laser_ortho_projector/laser_ortho_projector.h"

namespace scan_tools {

LaserOrthoProjector::LaserOrthoProjector (ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private)
{
  ROS_INFO ("Starting LaserOrthoProjector");

  initialized_ = false;

  cloud_ = boost::make_shared<PointCloudT>();

  // **** parameters

  if (!nh_private_.getParam ("fixed_frame", world_frame_))
    world_frame_ = "world";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!nh_private_.getParam ("ortho_frame", ortho_frame_))
    ortho_frame_ = "base_ortho";

  // **** subscribe to laser scan messages

  scan_subscriber_ = nh_.subscribe(
    scan_topic_, 10, &LaserOrthoProjector::scanCallback, this);

  // **** advertise orthogonal scan
  cloud_publisher_ = nh_.advertise<PointCloudT>(
    cloud_topic_, 10);
}

LaserOrthoProjector::~LaserOrthoProjector ()
{

}

void LaserOrthoProjector::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  if(!initialized_)
  {
    initialized_ = getBaseToLaserTf(scan_msg);

    if (initialized_) createCache(scan_msg);
    else return;
  }

  // **** obtain transform between fixed and base frame

  tf::StampedTransform world_to_base_tf;

  try
  {
    tf_listener_.lookupTransform (
      world_frame_, base_frame_, scan_msg->header.stamp, world_to_base_tf);
  }
  catch (tf::TransformException ex)
  {
    // transform unavailable - skip scan
    ROS_WARN ("Skipping scan (%s)", ex.what ());
    return;
  }
  btTransform world_to_base = world_to_base_tf;

  double roll, pitch, yaw;
  btMatrix3x3 m (world_to_base.getRotation ());
  m.getRPY (roll, pitch, yaw);

  // **** calculate and publish transform between world and ortho frames

  btTransform world_to_ortho;

  btQuaternion rotation;
  rotation.setRPY (0.0, 0.0, yaw);
  world_to_ortho.setRotation (rotation);

  btVector3 origin;
  origin.setValue (world_to_base.getOrigin().getX(),
                   world_to_base.getOrigin().getY(),
                   0.0);
  world_to_ortho.setOrigin (origin);

  tf::StampedTransform world_to_ortho_tf(world_to_ortho, scan_msg->header.stamp, world_frame_, ortho_frame_);
  tf_broadcaster_.sendTransform (world_to_ortho_tf);

  // **** build and publish projected cloud

  cloud_->points.clear();
  cloud_->header = scan_msg->header;
  cloud_->header.frame_id = ortho_frame_;

  for (unsigned int i = 0; i < scan_msg->ranges.size (); i++)
  {
    double r = scan_msg->ranges[i];

    if (r > scan_msg->range_min)
    {
      btVector3 p(r * a_cos_[i], r * a_sin_[i], 0.0);
      p = world_to_ortho.inverse() * world_to_base * base_to_laser_ * p;
      p.setZ(0.0);

      PointT point;
      point.x = p.getX();
      point.y = p.getY();
      point.z = p.getZ();
      cloud_->points.push_back(point);
    }
  }

  cloud_publisher_.publish (cloud_);
}

bool LaserOrthoProjector::getBaseToLaserTf (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  tf::StampedTransform base_to_laser_tf;
  try
  {
    tf_listener_.waitForTransform(
      base_frame_, scan_msg->header.frame_id, scan_msg->header.stamp, ros::Duration(1.0));
    tf_listener_.lookupTransform (
      base_frame_, scan_msg->header.frame_id, scan_msg->header.stamp, base_to_laser_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("LaserOrthoProjector: Could not get initial laser transform(%s)", ex.what());
    return false;
  }
  base_to_laser_ = base_to_laser_tf;

  return true;
}

void LaserOrthoProjector::createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  a_cos_.clear();
  a_sin_.clear();

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
    a_cos_.push_back(cos(angle));
    a_sin_.push_back(sin(angle));
  }
}

} //namespace scan_tools
