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
