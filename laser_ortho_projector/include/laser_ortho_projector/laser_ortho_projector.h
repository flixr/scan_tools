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

#ifndef LASER_ORTHO_PROJECTOR_LASER_ORTHO_PROJECTOR_H
#define LASER_ORTHO_PROJECTOR_LASER_ORTHO_PROJECTOR_H


#include <limits>
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

namespace scan_tools {

static const std::string scan_topic_ = "scan";
static const std::string imu_topic_  = "imu";
static const std::string cloud_topic_ = "cloud_ortho";

class LaserOrthoProjector
{
  typedef pcl::PointXYZ           PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;

  private:

    // **** ROS-related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher cloud_publisher_;
    ros::Subscriber scan_subscriber_;
    ros::Subscriber imu_subscriber_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    // **** paramaters

    std::string world_frame_;
    std::string base_frame_;
    std::string ortho_frame_;
    bool publish_tf_;
    bool use_imu_;

    // **** state variables

    bool initialized_;

    std::vector<double> a_sin_;
    std::vector<double> a_cos_;

    sensor_msgs::Imu latest_imu_msg_;

    PointT nan_point_;

    btTransform base_to_laser_; // static, cached

    PointCloudT::Ptr cloud_;

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void imuCallback (const sensor_msgs::Imu::ConstPtr& imu_msg);
    bool getBaseToLaserTf (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg);

  public:

    LaserOrthoProjector (ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~ LaserOrthoProjector ();
};

} // namespace scan_tools

#endif // LASER_ORTHO_PROJECTOR_LASER_ORTHO_PROJECTOR_H
