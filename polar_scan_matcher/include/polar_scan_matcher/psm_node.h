/*
*  Polar Scan Matcher
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
*  Polar Scan Matcher [1] algorithm written by A. Diosi
*
*  [1] A. Diosi and L. Kleeman, "Laser Scan Matching in Polar Coordinates with 
*  Application to SLAM " Proceedings of 2005 IEEE/RSJ International Conference 
*  on Intelligent Robots and Systems, August, 2005, Edmonton, Canada
*/

#ifndef POLAR_SCAN_MATCHER_PSM_NODE_H
#define POLAR_SCAN_MATCHER_PSM_NODE_H

#include <sys/time.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>

#include "polar_scan_matcher/polar_match.h"

const std::string imuTopic_  = "imu";
const std::string scanTopic_ = "scan";
const std::string poseTopic_ = "pose2D";

const double ROS_TO_PM = 100.0;   // convert from cm to m

class PSMNode
{
  private:

    ros::Subscriber scanSubscriber_;
    ros::Subscriber imuSubscriber_;
    ros::Publisher  posePublisher_;

    tf::TransformBroadcaster tfBroadcaster_;
    tf::TransformListener    tfListener_;
    btTransform prevWorldToBase_;
    btTransform baseToLaser_;
    btTransform laserToBase_;

    bool initialized_;
    double totalDuration_;
    int scansCount_;

    PolarMatcher matcher_;
    PMScan * prevPMScan_;

    boost::mutex imuMutex_;
    double prevImuAngle_;   // the yaw angle when we last perfomred a scan match
    double currImuAngle_;   // the most recent yaw angle we have received

    // **** parameters

    bool   publishTf_;
    bool   publishPose_;
    bool   useTfOdometry_;
    bool   useImuOdometry_;

    int    minValidPoints_;
    int    searchWindow_;
    double maxError_;
    int    maxIterations_;
    double stopCondition_;

    std::string worldFrame_;
    std::string baseFrame_;
    std::string laserFrame_;

    void getParams();
    bool initialize(const sensor_msgs::LaserScan& scan);

    void imuCallback (const sensor_msgs::Imu& imuMsg);
    void scanCallback (const sensor_msgs::LaserScan& scan);

    void publishTf(const btTransform& transform, 
                   const ros::Time& time);
    void publishPose(const btTransform& transform);

    void rosToPMScan(const sensor_msgs::LaserScan& scan, 
                     const btTransform& change,
                           PMScan* pmScan);
    void pose2DToTf(const geometry_msgs::Pose2D& pose, btTransform& t);
    void tfToPose2D(const btTransform& t, geometry_msgs::Pose2D& pose);
    void getCurrentEstimatedPose(btTransform& worldToBase, 
                                 const sensor_msgs::LaserScan& scanMsg);

  public:

    PSMNode();
    virtual ~PSMNode();
};

#endif // POLAR_SCAN_MATCHER_PSM_NODE_H
