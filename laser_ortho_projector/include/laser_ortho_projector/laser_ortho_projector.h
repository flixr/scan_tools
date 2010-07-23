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

#ifndef LASER_ORTHO_PROJECTION_LASER_ORTHO_PROJECTION_H
#define LASER_ORTHO_PROJECTION_LASER_ORTHO_PROJECTION_H

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_ortho_projector/LaserScanWithAngles.h>

static const std::string scanTopic_ = "scan";
static const std::string scanOrthoTopic_ = "scan_ortho";
static const std::string cloudOrthoTopic_ = "cloud_ortho";

class LaserOrthoProjector
{
public:

  LaserOrthoProjector ();
  virtual ~ LaserOrthoProjector ();

private:

  bool useImu_;
  ros::Subscirber scanSubscriber_;

  // paramaters

  std::string worldFrame_;
  std::string laserOrthoFrame_;

  bool publishCloud_;
  int tfTolerance_;

  // transforms

  tf::StampedTransform worldToLaser_;
  double roll_, pitch_, yaw_;

  // publishers & subscirbers
  ros::Publisher scanPublisher_;
  ros::Publisher cloudPublisher_;

  message_filters::Subscriber < sensor_msgs::LaserScan > *scanFilterSub_;
  tf::MessageFilter < sensor_msgs::LaserScan > *scanFilter_;

  tf::TransformListener tfListener_;
  tf::TransformBroadcaster tfBroadcaster_;

  void scanCallback (const sensor_msgs::LaserScanConstPtr & scan);
  void publishWorldToLaserOrthoTf (ros::Time time);
  void publishScanOrtho (const sensor_msgs::LaserScanConstPtr & scan,
                         laser_ortho_projector::LaserScanWithAngles & scanOrtho);
  void publishCloudOrtho (const laser_ortho_projector::LaserScanWithAngles & scanOrtho);
};

#endif // LASER_ORTHO_PROJECTION_LASER_ORTHO_PROJECTION_H
