/*
*  Laser Scan Matcher
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
*
*  This package uses Canonical Scan Matcher [1], written by 
*  Andrea Censi
*
*  [1] A. Censi, "An ICP variant using a point-to-line metric" 
*  Proceedings of the IEEE International Conference 
*  on Robotics and Automation (ICRA), 2008
*/

#ifndef LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
#define LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains
#undef min 
#undef max

namespace scan_tools
{

// inputs
const std::string scan_topic_  = "scan";
const std::string cloud_topic_ = "cloud";
const std::string odom_topic_  = "odom";
const std::string imu_topic_   = "imu";

// outputs
const std::string pose_topic_ = "pose2D";

typedef pcl::PointXYZ           PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class LaserScanMatcher
{
  private:

    // **** ros  

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber scan_subscriber_;
    ros::Subscriber cloud_subscriber_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber imu_subscriber_;

    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    tf::Transform base_to_laser_;
    tf::Transform laser_to_base_;

    ros::Publisher  test_pub_;
    ros::Publisher  pose_publisher_;
    ros::Publisher  vel_publisher_;

    // **** parameters

    std::string base_frame_;
    std::string fixed_frame_;
    double cloud_range_min_;
    double cloud_range_max_;
    bool publish_tf_;
    bool publish_pose_;

    bool use_cloud_input_;

    // **** What predictions are available to speed up the ICP?
    // 1) imu - [theta] from imu yaw angle - /odom topic
    // 2) odom - [x, y, theta] from wheel odometry - /imu topic
    // 3) alpha_beta - [x, y, theta] from simple tracking filter - no topic req.
    // If more than one is enabled, priority is imu > odom > alpha_beta

    bool use_imu_;
    bool use_odom_;
    bool use_alpha_beta_;

    double alpha_;
    double beta_;

    // **** state variables

    bool initialized_;

    bool received_imu_;
    bool received_odom_;

    boost::mutex mutex_;

    geometry_msgs::Pose2D::Ptr pose_msg_;

    tf::Transform w2b_; // world-to-base tf (pose of base frame)

    double v_x_;  // velocity estimated by the alpha-beta tracker
    double v_y_;  
    double v_a_;

    ros::Time last_icp_time_;

    double latest_imu_yaw_;
    double last_imu_yaw_;

    nav_msgs::Odometry latest_odom_;
    nav_msgs::Odometry last_odom_;

    std::vector<double> a_cos_;
    std::vector<double> a_sin_;

    sm_params input_;
    sm_result output_;
    LDP prev_ldp_scan_;

    // **** methods

    void processScan(LDP& curr_ldp_scan, const ros::Time& time);

    void laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                              LDP& ldp);
    void PointCloudToLDP(const PointCloudT::ConstPtr& cloud,
                               LDP& ldp);

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void cloudCallback (const PointCloudT::ConstPtr& cloud);

    void odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg);
    void imuCallback (const sensor_msgs::ImuPtr& imu_msg);

    void createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    bool getBaseToLaserTf (const std::string& frame_id);
    void initParams();

    void getPrediction(double& pr_ch_x, double& pr_ch_y, 
                       double& pr_ch_a, double dt);

    double getYawFromQuaternion(const geometry_msgs::Quaternion& quaternion);
    double getYawFromQuaternion(const tf::Quaternion& quaternion);
    void createTfFromXYTheta(double x, double y, double theta, tf::Transform& t);

  public:

    LaserScanMatcher(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~LaserScanMatcher();
};

} //namespace

#endif // LASER_SCAN_MATCHER_LASER_SCAN_MATCHER_H
