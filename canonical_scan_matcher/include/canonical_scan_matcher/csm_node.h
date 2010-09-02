/*
*  Canonical Scan Matcher
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
*  This is a wrapper around Canonical Scan Matcher [1], written by 
*  Andrea Censi
*
*  [1] A. Censi, "An ICP variant using a point-to-line metric" 
*  Proceedings of the IEEE International Conference 
*  on Robotics and Automation (ICRA), 2008
*/

#ifndef CANONICAL_SCAN_MATCHER_CSM_NODE_H
#define CANONICAL_SCAN_MATCHER_CSM_NODE_H

#include <sys/time.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>

#include <csm/csm_all.h>

#ifdef USE_PROJECTED_SCANS
  #include <laser_ortho_projector/LaserScanWithAngles.h>
  typedef laser_ortho_projector::LaserScanWithAngles Scan;
#else
  #include <sensor_msgs/LaserScan.h>
  typedef sensor_msgs::LaserScan Scan;
#endif


const std::string imuTopic_  = "imu";
const std::string scanTopic_ = "scan";
const std::string poseTopic_ = "pose2D";

class CSMNode
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

    sm_params input_;
    sm_result output_;
    LDP prevLDPScan_;

    boost::mutex imuMutex_;
    double prevImuAngle_;   // the yaw angle when we last perfomred a scan match
    double currImuAngle_;   // the most recent yaw angle we have received

    // **** parameters
 
    bool   publishTf_;
    bool   publishPose_;
    bool   useTfOdometry_;
    bool   useImuOdometry_;

    std::string worldFrame_;
    std::string baseFrame_;
    std::string laserFrame_;

    void getParams();
    bool initialize(const Scan& scan);

    void imuCallback (const sensor_msgs::Imu& imuMsg);
    void scanCallback (const Scan& scan);

    void publishTf(const btTransform& transform, 
                   const ros::Time& time);
    void publishPose(const btTransform& transform);

    LDP rosToLDPScan(const Scan& scan,
                     const geometry_msgs::Pose2D& laserPose);

    void tfToPose2D(const btTransform& t, geometry_msgs::Pose2D& pose);
    void pose2DToTf(const geometry_msgs::Pose2D& pose, btTransform& t);
    void getCurrentEstimatedPose(btTransform& worldToBase, 
                                 const Scan& scanMsg);

  public:

    CSMNode();
    virtual ~CSMNode();
};

#endif // CANONICAL_SCAN_MATCHER_CSM_NODE_H
