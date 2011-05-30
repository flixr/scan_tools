#ifndef CANONICAL_SCAN_MATCHER_XY_CANONICAL_SCAN_MATCHER_H
#define CANONICAL_SCAN_MATCHER_XY_CANONICAL_SCAN_MATCHER_H

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

const std::string scan_topic_  = "scan";
const std::string cloud_topic_ = "cloud";

const std::string odom_topic_ = "odom";
const std::string imu_topic_  = "imu";

const std::string pose_topic_ = "laser_odom";
const std::string vel_topic_  = "/mav/vel";

typedef pcl::PointXYZ           PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class CanonicalScanMatcher
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
    double range_min_;
    double range_max_;
    bool publish_tf_;
    bool publish_pose_;

    bool use_cloud_input_;

    double min_cloud_angle_; // needed when using point cloud input
    double max_cloud_angle_;

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

    int received_imu_;
    int received_odom_;

    boost::mutex mutex_;

    geometry_msgs::Pose2D::Ptr pose_msg_;
    geometry_msgs::Twist::Ptr twist_msg_;

    double x_;
    double y_;
    double theta_;
    double cos_theta_, sin_theta_;

    double v_x_;
    double v_y_;
    double v_theta_;

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

    void broadcastTf(const ros::Time& time);

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

  public:

    CanonicalScanMatcher(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~CanonicalScanMatcher();
};

} //namespace


#endif // CANONICAL_SCAN_MATCHER_XY_CANONICAL_SCAN_MATCHER_H
