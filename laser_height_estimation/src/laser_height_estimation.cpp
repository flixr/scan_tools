#include "laser_height_estimation/laser_height_estimation.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_height_estimation");
  LaserHeightEstimation laser_height_estimation;
  ros::spin();
  return 0;
}

LaserHeightEstimation::LaserHeightEstimation()
{
  ROS_INFO("Starting LaserHeightEstimation"); 

  initialized_  = false;
  floor_height_ = 0.0;
  prev_height_  = 0.0;

  height_to_base_msg_      = boost::make_shared<std_msgs::Float64>();
  height_to_footprint_msg_ = boost::make_shared<std_msgs::Float64>();

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // **** parameters
  
  if (!nh_private.getParam ("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!nh_private.getParam ("footprint_frame", footprint_frame_))
    footprint_frame_ = "base_footprint";
  if (!nh_private.getParam ("min_values", min_values_))
    min_values_ = 5;
  if (!nh_private.getParam ("max_stdev", max_stdev_))
    max_stdev_ = 0.10;
  if (!nh_private.getParam ("max_height_jump", max_height_jump_))
    max_height_jump_ = 0.25;
  
  // **** subscribers

  scan_subscriber_ = nh.subscribe(
    scan_topic_, 5, &LaserHeightEstimation::scanCallback, this);
  imu_subscriber_ = nh.subscribe(
    imu_topic_, 5, &LaserHeightEstimation::imuCallback, this);

  // **** publishers

  height_to_base_publisher_ = nh.advertise<std_msgs::Float64>(
    height_to_base_topic_, 5);

  height_to_footprint_publisher_ = nh.advertise<std_msgs::Float64>(
    height_to_footprint_topic_, 5);
}

LaserHeightEstimation::~LaserHeightEstimation()
{
  ROS_INFO("Destroying LaserHeightEstimation"); 
}

void LaserHeightEstimation::imuCallback (const sensor_msgs::ImuPtr& imu_msg)
{
  imu_transform_.setRotation(
    btQuaternion(imu_msg->orientation.x, imu_msg->orientation.y,
                 imu_msg->orientation.z, imu_msg->orientation.w));
  
}

void LaserHeightEstimation::scanCallback (const sensor_msgs::LaserScanPtr& scan_msg)
{
  if (!initialized_)
  {
    // if this is the first scan, lookup the static base to laser tf
    // if tf is not available yet, skip this scan
    if (!setBaseToLaserTf(scan_msg)) return;
    initialized_ = true;
  }

  // **** get required transforms

  btTransform rotated_laser     = imu_transform_ * base_to_laser_;
  btTransform rotated_footprint = imu_transform_ * base_to_footprint_;
 
  // **** get vector of height values
  
  std::vector<double> values;
  for(unsigned int i = 0; i < scan_msg->ranges.size(); i++)
  {
    if (scan_msg->ranges[i] > scan_msg->range_min && scan_msg->ranges[i] < scan_msg->range_max)
    {
      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      btVector3 v(cos(angle)*scan_msg->ranges[i], sin(angle)*scan_msg->ranges[i], 0.0);
      btVector3 p = rotated_laser * v;
      
      values.push_back(p.getZ());
    }
  }

  if (values.size() < min_values_)
  {
    ROS_WARN("Not enough valid values to determine height, skipping (%d collected, %d needed)",
      values.size(), min_values_);
    return;
  }

  // **** get mean and standard dev

  double mean_value, stdev_value;
  getStats(values, mean_value, stdev_value);

  if (stdev_value > max_stdev_)
  {
    ROS_WARN("Stdev of height readings too big to determine height, skipping (stdev is %f, max is %f)",
      stdev_value, max_stdev_);
    return;
  }

  // **** estimate height (to base and to footprint)

  double height_to_base = 0.0 - mean_value + floor_height_;
  double height_to_footprint = rotated_footprint.getOrigin().getZ() - mean_value + floor_height_; 
 
  // **** check for discontinuity
 
  double height_jump = prev_height_ - height_to_base;

  if (fabs(height_jump) > max_height_jump_)
  {
    ROS_WARN("Laser Height Estimation: Floor Discontinuity detected");
    floor_height_ += height_jump;
    height_to_base += height_jump;
    height_to_footprint += height_jump;
  }

  prev_height_ = height_to_base;

  // **** publish height message

  height_to_base_msg_->data = height_to_base;
  height_to_footprint_msg_->data = height_to_footprint;
  height_to_base_publisher_.publish(height_to_base_msg_);
  height_to_footprint_publisher_.publish(height_to_footprint_msg_);
}

bool LaserHeightEstimation::setBaseToLaserTf(const sensor_msgs::LaserScanPtr& scan_msg)
{
  // **** get transform base to laser

  tf::StampedTransform base_to_laser_tf;
  try
  {
     tf_listener_.lookupTransform (
       base_frame_, scan_msg->header.frame_id, scan_msg->header.stamp, base_to_laser_tf);
  }
  catch (tf::TransformException ex)
  {
    // transform unavailable - skip scan
    ROS_WARN("LHE: Transform unavailable, skipping scan (%s)", ex.what());
    return false;
  }
  
  base_to_laser_ = base_to_laser_tf;

  // **** get transform base to base_footprint

  tf::StampedTransform base_to_footprint_tf;
  try
  {
     tf_listener_.lookupTransform (
       base_frame_, footprint_frame_, ros::Time::now(), base_to_footprint_tf);
  }
  catch (tf::TransformException ex)
  {
    // transform unavailable - skip scan
    ROS_WARN("LHE: Transform unavailable, skipping scan (%s)", ex.what());
    return false;
  }
  
  base_to_footprint_ = base_to_footprint_tf;

  return true;
}

void LaserHeightEstimation::getStats(const std::vector<double> values, double& ave, double& stdev)
{
  double sum   = 0.0;
  double sumsq = 0.0;

  for (size_t i = 0; i < values.size(); ++i)
    sum += values[i];

  ave = sum/values.size();

  for (size_t i = 0; i < values.size(); ++i)
    sumsq += (values[i] - ave) * (values[i] - ave);

  stdev = sqrt(sumsq/values.size());
}
