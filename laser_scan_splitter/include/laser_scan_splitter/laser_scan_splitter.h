/*
*  Laser Scan Splitter
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

#ifndef LASER_SCAN_SPLITTER_LASER_SCAN_SPLITTER_H
#define LASER_SCAN_SPLITTER_LASER_SCAN_SPLITTER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

const std::string scan_topic_ = "scan";

class LaserScanSplitter
{
  private:

    // **** ROS-related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber scan_subscriber_;
    std::vector<ros::Publisher> scan_publishers_;

    // **** paramaters

    std::vector<std::string> published_scan_topics_;
    std::vector<std::string> published_laser_frames_;
    std::vector<int> sizes_;

    // **** state variables

    unsigned int size_sum_;

    // **** member functions

    void scanCallback (const sensor_msgs::LaserScanConstPtr& scan_msg);
    void tokenize (const std::string& str, std::vector<std::string>& tokens);

  public:

    LaserScanSplitter (ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~ LaserScanSplitter ();
};

#endif // LASER_SCAN_SPLITTER_LASER_SCAN_SPLITTER_H
