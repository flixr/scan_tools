/*
 * Copyright (c) 2010, 2011, Ivan Dryanovski, William Morris
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the CCNY Robotics Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "laser_ortho_projector/laser_ortho_projector.h"

namespace scan_tools {

LaserOrthoProjector::LaserOrthoProjector (ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private)
{
  ROS_INFO ("Starting LaserOrthoProjector");

  initialized_ = false;

  nan_point_.x = std::numeric_limits<float>::quiet_NaN();
  nan_point_.y = std::numeric_limits<float>::quiet_NaN();
  nan_point_.z = std::numeric_limits<float>::quiet_NaN();

  // **** parameters

  if (!nh_private_.getParam ("fixed_frame", world_frame_))
    world_frame_ = "/world";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/base_link";
  if (!nh_private_.getParam ("ortho_frame", ortho_frame_))
    ortho_frame_ = "/base_ortho";
  if (!nh_private_.getParam ("publish_tf", publish_tf_))
    publish_tf_ = false;
  if (!nh_private_.getParam ("use_imu", use_imu_))
    use_imu_ = true;

  // **** subscribe to laser scan messages

  scan_subscriber_ = nh_.subscribe(
    scan_topic_, 10, &LaserOrthoProjector::scanCallback, this);

  if (use_imu_)
  {
    imu_subscriber_ = nh_.subscribe(
      imu_topic_, 10, &LaserOrthoProjector::imuCallback, this);
  }

  // **** advertise orthogonal scan

  cloud_publisher_ = nh_.advertise<PointCloudT>(
    cloud_topic_, 10);
}

LaserOrthoProjector::~LaserOrthoProjector ()
{

}

void LaserOrthoProjector::imuCallback (const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  latest_imu_msg_ = *imu_msg;
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

  btTransform world_to_base;

  if(use_imu_)
  {
    world_to_base.setIdentity();
    btQuaternion q;
    tf::quaternionMsgToTF(latest_imu_msg_.orientation, q);
    world_to_base.setRotation(q);
  }
  else
  {
    tf::StampedTransform world_to_base_tf;

    try
    {
      tf_listener_.waitForTransform (
        world_frame_, base_frame_, scan_msg->header.stamp, ros::Duration(0.5));
      tf_listener_.lookupTransform (
        world_frame_, base_frame_, scan_msg->header.stamp, world_to_base_tf);
    }
    catch (tf::TransformException ex)
    {
      // transform unavailable - skip scan
      ROS_WARN ("Skipping scan (%s)", ex.what ());
      return;
    }
    world_to_base = world_to_base_tf;
  }

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

  if (publish_tf_)
  {
    tf::StampedTransform world_to_ortho_tf(world_to_ortho, scan_msg->header.stamp, world_frame_, ortho_frame_);
    tf_broadcaster_.sendTransform (world_to_ortho_tf);
  }

  // **** build and publish projected cloud

  PointCloudT::Ptr cloud = boost::make_shared<PointCloudT>();

  cloud->header = scan_msg->header;
  cloud->header.frame_id = ortho_frame_;

  for (unsigned int i = 0; i < scan_msg->ranges.size(); i++)
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
      cloud->points.push_back(point);
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true; // no nan's present 

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
