#ifndef CANONICAL_SCAN_MATCHER_XY_CANONICAL_SCAN_MATCHER_H
#define CANONICAL_SCAN_MATCHER_XY_CANONICAL_SCAN_MATCHER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains
#undef min 
#undef max

namespace scan_matcher
{

const std::string scan_topic_ = "scan";
const std::string imu_topic_  = "imu";
const std::string pose_topic_ = "laser_odom";

typedef pcl::PointXYZ           PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const std::string pub_cloud_topic_ = "test_cloud";

class CanonicalScanMatcher
{
  private:

    // **** ros  

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber scan_subscriber_;
    ros::Subscriber imu_subscriber_;

    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    tf::Transform base_to_laser_;
    tf::Transform laser_to_base_;

    ros::Publisher  test_pub_;
    ros::Publisher  pose_publisher_;

    // **** parameters

    std::string base_frame_;
    std::string fixed_frame_;
    double range_min_;
    double range_max_;
    bool publish_tf_;
    bool publish_pose_;

    // **** state variables

    bool initialized_;

    geometry_msgs::Pose2D::Ptr pose_msg_;

    double x_;
    double y_;
    double theta_;

    double v_x_;
    double v_y_;
    double v_theta_;

    bool use_alpha_beta_;
    double alpha_;
    double beta_;

    ros::Time last_icp_time_;

    double latest_imu_roll_;
    double latest_imu_pitch_;
    double latest_imu_yaw_;

    std::vector<double> a_cos_;
    std::vector<double> a_sin_;

    sm_params input_;
    sm_result output_;
    LDP prev_ldp_scan_;

    // **** methods

    void broadcastTf(const ros::Time& time);

    void processScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    void laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                              LDP& ldp);

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void imuCallback (const sensor_msgs::ImuPtr& imu_msg);

    void createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    bool getBaseToLaserTf (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void initParams();

  public:

    CanonicalScanMatcher(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~CanonicalScanMatcher();
};

} //namespace


#endif // CANONICAL_SCAN_MATCHER_XY_CANONICAL_SCAN_MATCHER_H
