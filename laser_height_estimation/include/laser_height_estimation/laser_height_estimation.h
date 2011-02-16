#ifndef LASER_HEIGHT_ESTIMATION_LASER_HEIGHT_ESTIMATION_H
#define LASER_HEIGHT_ESTIMATION_LASER_HEIGHT_ESTIMATION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <boost/thread/mutex.hpp>

#include <std_msgs/Float64.h>

const std::string scan_topic_                = "scan";
const std::string height_to_base_topic_      = "height_to_base";
const std::string height_to_footprint_topic_ = "height_to_footprint";
const std::string imu_topic_                 = "imu";

class LaserHeightEstimation
{
  private:

    // **** ros-related variables

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber imu_subscriber_;
    ros::Subscriber scan_subscriber_;
    ros::Publisher  height_to_base_publisher_;
    ros::Publisher  height_to_footprint_publisher_;
    tf::TransformListener tf_listener_;

    // **** state variables

    bool initialized_;
    double floor_height_;
    double prev_height_;

    btTransform base_to_laser_;
    btTransform base_to_footprint_;
    btTransform imu_transform_;
   
    std_msgs::Float64Ptr height_to_base_msg_;
    std_msgs::Float64Ptr height_to_footprint_msg_;

    // **** parameters
  
    std::string base_frame_;
    std::string footprint_frame_;
    int min_values_;
    double max_stdev_;
    double max_height_jump_;

    // **** member functions

    void scanCallback (const sensor_msgs::LaserScanPtr& scan_msg);
    void imuCallback  (const sensor_msgs::ImuPtr&       imu_msg);
    bool setBaseToLaserTf(const sensor_msgs::LaserScanPtr& scan_msg);
    void getStats(const std::vector<double> values, double& ave, double& stdev);

  public:
  
    LaserHeightEstimation(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~LaserHeightEstimation();
};

#endif // LASER_HEIGHT_ESTIMATION_LASER_HEIGHT_ESTIMATION_H

