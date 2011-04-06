#include "canonical_scan_matcher_xy/canonical_scan_matcher.h"

namespace scan_matcher
{

CanonicalScanMatcher::CanonicalScanMatcher(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  ROS_INFO("Starting ScanMatcher"); 

  // **** init parameters

  initParams();

  // **** state variables 

  initialized_ = false;
  latest_imu_roll_ = 0;
  latest_imu_pitch_ = 0;
  latest_imu_yaw_ = 0;

  x_ = 0;
  y_ = 0;
  theta_ = 0;

  v_x_ = 0;
  v_y_ = 0;
  v_theta_ = 0;

  pose_msg_ = boost::make_shared<geometry_msgs::Pose2D>();

  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0; 
  input_.laser[2] = 0.0; 

  input_.min_reading = range_min_;
  input_.max_reading = range_max_;

  // *** subscribers

  scan_subscriber_ = nh_.subscribe(
    scan_topic_, 1, &CanonicalScanMatcher::scanCallback, this);

  imu_subscriber_ = nh_.subscribe(
    imu_topic_, 1, &CanonicalScanMatcher::imuCallback, this);

  // **** test
  test_pub_ = nh_.advertise<PointCloudT>(pub_cloud_topic_, 1);

  // **** pose publisher
  pose_publisher_  = nh_.advertise<geometry_msgs::Pose2D>(
    pose_topic_, 5);
}

CanonicalScanMatcher::~CanonicalScanMatcher()
{

}

void CanonicalScanMatcher::initParams()
{
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "world"; 
  if (!nh_private_.getParam ("range_min", range_min_))
    range_min_ = 0.1;
  if (!nh_private_.getParam ("range_max", range_max_))
    range_max_ = 50.0;
  if (!nh_private_.getParam ("publish_tf", publish_tf_))
    publish_tf_ = true;
  if (!nh_private_.getParam ("publish_pose", publish_pose_))
    publish_pose_ = true;
  if (!nh_private_.getParam ("use_alpha_beta", use_alpha_beta_))
    use_alpha_beta_ = true;
  if (!nh_private_.getParam ("alpha", alpha_))
    alpha_ = 0.5;
  if (!nh_private_.getParam ("beta", beta_))
    beta_ = 0.5;

 // **** CSM parameters - comments copied from algos.h (by Andrea Censi)

  // Maximum angular displacement between scans
  if (!nh_private_.getParam ("max_angular_correction_deg", input_.max_angular_correction_deg))
    input_.max_angular_correction_deg = 45.0;

  // Maximum translation between scans (m)
  if (!nh_private_.getParam ("max_linear_correction", input_.max_linear_correction))
    input_.max_linear_correction = 0.50;

  // Maximum ICP cycle iterations
  if (!nh_private_.getParam ("max_iterations", input_.max_iterations))
    input_.max_iterations = 10;

  // A threshold for stopping (m)
  if (!nh_private_.getParam ("epsilon_xy", input_.epsilon_xy))
    input_.epsilon_xy = 0.001;

  // A threshold for stopping (rad)
  if (!nh_private_.getParam ("epsilon_theta", input_.epsilon_theta))
    input_.epsilon_theta = 0.00872;

  // Maximum distance for a correspondence to be valid
  if (!nh_private_.getParam ("max_correspondence_dist", input_.max_correspondence_dist))
    input_.max_correspondence_dist = 0.5;

  // Noise in the scan (m)
  if (!nh_private_.getParam ("sigma", input_.sigma))
    input_.sigma = 0.010;

  // Use smart tricks for finding correspondences.
  if (!nh_private_.getParam ("use_corr_tricks", input_.use_corr_tricks))
    input_.use_corr_tricks = 1;

  // Restart: Restart if error is over threshold
  if (!nh_private_.getParam ("restart", input_.restart))
    input_.restart = 0;

  // Restart: Threshold for restarting
  if (!nh_private_.getParam ("restart_threshold_mean_error", input_.restart_threshold_mean_error))
    input_.restart_threshold_mean_error = 0.01;

  // Restart: displacement for restarting. (m)
  if (!nh_private_.getParam ("restart_dt", input_.restart_dt))
    input_.restart_dt = 1.0;

  // Restart: displacement for restarting. (rad)
  if (!nh_private_.getParam ("restart_dtheta", input_.restart_dtheta))
    input_.restart_dtheta = 0.1;

  // Max distance for staying in the same clustering
  if (!nh_private_.getParam ("clustering_threshold", input_.clustering_threshold))
    input_.clustering_threshold = 0.25;

  // Number of neighbour rays used to estimate the orientation
  if (!nh_private_.getParam ("orientation_neighbourhood", input_.orientation_neighbourhood))
    input_.orientation_neighbourhood = 10;

  // If 0, it's vanilla ICP
  if (!nh_private_.getParam ("use_point_to_line_distance", input_.use_point_to_line_distance))
    input_.use_point_to_line_distance = 1;

  // Discard correspondences based on the angles
  if (!nh_private_.getParam ("do_alpha_test", input_.do_alpha_test))
    input_.do_alpha_test = 0;

  // Discard correspondences based on the angles - threshold angle, in degrees
  if (!nh_private_.getParam ("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
    input_.do_alpha_test_thresholdDeg = 20.0;

  // Percentage of correspondences to consider: if 0.9,
	// always discard the top 10% of correspondences with more error
  if (!nh_private_.getParam ("outliers_maxPerc", input_.outliers_maxPerc))
    input_.outliers_maxPerc = 0.90;

  // Parameters describing a simple adaptive algorithm for discarding.
	//  1) Order the errors.
	//	2) Choose the percentile according to outliers_adaptive_order.
	//	   (if it is 0.7, get the 70% percentile)
	//	3) Define an adaptive threshold multiplying outliers_adaptive_mult
	//	   with the value of the error at the chosen percentile.
	//	4) Discard correspondences over the threshold.
	//	This is useful to be conservative; yet remove the biggest errors.
  if (!nh_private_.getParam ("outliers_adaptive_order", input_.outliers_adaptive_order))
    input_.outliers_adaptive_order = 0.7;

  if (!nh_private_.getParam ("outliers_adaptive_mult", input_.outliers_adaptive_mult))
    input_.outliers_adaptive_mult = 2.0;

  //If you already have a guess of the solution, you can compute the polar angle
	//	of the points of one scan in the new position. If the polar angle is not a monotone
	//	function of the readings index, it means that the surface is not visible in the 
	//	next position. If it is not visible, then we don't use it for matching.
  if (!nh_private_.getParam ("do_visibility_test", input_.do_visibility_test))
    input_.do_visibility_test = 0;

  // no two points in laser_sens can have the same corr.
  if (!nh_private_.getParam ("outliers_remove_doubles", input_.outliers_remove_doubles))
    input_.outliers_remove_doubles = 1;

  // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
  if (!nh_private_.getParam ("do_compute_covariance", input_.do_compute_covariance))
    input_.do_compute_covariance = 0;

  // Checks that find_correspondences_tricks gives the right answer
  if (!nh_private_.getParam ("debug_verify_tricks", input_.debug_verify_tricks))
    input_.debug_verify_tricks = 0;

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the 
  // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
  if (!nh_private_.getParam ("use_ml_weights", input_.use_ml_weights))
    input_.use_ml_weights = 0;

  // If 1, the field 'readings_sigma' in the second scan is used to weight the 
  // correspondence by 1/sigma^2
  if (!nh_private_.getParam ("use_sigma_weights", input_.use_sigma_weights))
    input_.use_sigma_weights = 0;


  printf("input_.max_iterations: %d\n", input_.max_iterations);
}

void CanonicalScanMatcher::imuCallback (const sensor_msgs::ImuPtr& imu_msg)
{
  //@FIXME: mutex

  btQuaternion q;
	tf::quaternionMsgToTF(imu_msg->orientation, q);
  btMatrix3x3 m(q);
  mutex_.lock();
  m.getRPY(latest_imu_roll_, latest_imu_pitch_, latest_imu_yaw_);
  mutex_.unlock();
}


void CanonicalScanMatcher::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  // **** if first scan, cache the tf from base to the scanner
  
  if (!initialized_)
  {
    createCache(scan_msg);    // caches the sin and cos of all angles

    // cache the static tf from base to laser
    if (!getBaseToLaserTf(scan_msg))
    {
      ROS_WARN("ScanMatcher: Skipping scan");
      return;
    }

    laserScanToLDP(scan_msg, prev_ldp_scan_); 
    last_icp_time_ = ros::Time::now();
    last_imu_yaw_ = latest_imu_yaw_;
    initialized_ = true;
  }

  // **** process the laser scan

  struct timeval start_, end_;    // used for timing
  gettimeofday(&start_, NULL);

  processScan(scan_msg);
  
  gettimeofday(&end_, NULL);
  double dur = ((end_.tv_sec   * 1000000 + end_.tv_usec  ) - 
                (start_.tv_sec * 1000000 + start_.tv_usec)) / 1000.0;
  printf("ICP: %.1f ms \n", dur);
}

void CanonicalScanMatcher::processScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  // CSM is used in the following way:
  // The reference scan (prevLDPcan_) always has a pose of 0
  // The new scan (currLDPScan) has a pose equal to the movement
  // of the laser in the world frame since the last scan (btTransform change)
  // The computed correction is then propagated using the tf machinery

  // **** set up the "model" (previous scan)

  prev_ldp_scan_->odometry[0] = 0;
  prev_ldp_scan_->odometry[1] = 0;
  prev_ldp_scan_->odometry[2] = 0;

  prev_ldp_scan_->estimate[0] = 0;
  prev_ldp_scan_->estimate[1] = 0;
  prev_ldp_scan_->estimate[2] = 0;

  prev_ldp_scan_->true_pose[0] = 0;
  prev_ldp_scan_->true_pose[1] = 0;
  prev_ldp_scan_->true_pose[2] = 0;

  // **** set up the "data" (new scan)

  LDP curr_ldp_scan;
  laserScanToLDP(scan_msg, curr_ldp_scan);

  input_.laser_ref  = prev_ldp_scan_;
  input_.laser_sens = curr_ldp_scan;

  // **** estimated change since last scan

  ros::Time new_icp_time = ros::Time::now();
  ros::Duration dur = new_icp_time - last_icp_time_;

  double cos_theta = cos(theta_);
  double sin_theta = sin(theta_);

  double exp_ch_x, exp_ch_y, exp_ch_a;

  mutex_.lock();
  if(use_alpha_beta_)
  {
    exp_ch_x = v_x_     * dur.toSec();
    exp_ch_y = v_y_     * dur.toSec();
    exp_ch_a = v_theta_ * dur.toSec();

    input_.first_guess[0] = (cos_theta * exp_ch_x + sin_theta * exp_ch_y);
    input_.first_guess[1] = (sin_theta * exp_ch_x + cos_theta * exp_ch_y);
    input_.first_guess[2] = latest_imu_yaw_ - last_imu_yaw_; //@FIXME: lock
  }
  else
  {
    input_.first_guess[0] = 0;
    input_.first_guess[1] = 0;
    input_.first_guess[2] = latest_imu_yaw_ - last_imu_yaw_;
  }
  last_imu_yaw_ = latest_imu_yaw_;
  mutex_.unlock();

  // *** scan match - using icp (xy means x and y are already computed)

  sm_icp_xy(&input_, &output_);

  if (output_.valid) 
  {
    // **** calculate change in position of the laser

    double dx = (cos_theta * output_.x[0] - sin_theta * output_.x[1]);
    double dy = (sin_theta * output_.x[0] + cos_theta * output_.x[1]);
    double da = output_.x[2];

    if(use_alpha_beta_)
    {
      double r_x = dx - exp_ch_x;
      double r_y = dy - exp_ch_y;
      double r_a = da - exp_ch_a;

      x_     = (x_     + exp_ch_x) + alpha_ * r_x;
      y_     = (y_     + exp_ch_y) + alpha_ * r_y;
      theta_ = (theta_ + exp_ch_a) + alpha_ * r_a;

      v_x_     = v_x_     + (beta_ / dur.toSec()) * r_x;
      v_y_     = v_y_     + (beta_ / dur.toSec()) * r_y;
      v_theta_ = v_theta_ + (beta_ / dur.toSec()) * r_a;
    }
    else
    {
      x_     += dx;
      y_     += dy;
      theta_ += da;
    }

    // **** publish

    if (publish_pose_) 
    {
      pose_msg_->x = x_;
      pose_msg_->y = y_;
      pose_msg_->theta = theta_;

      pose_publisher_.publish(pose_msg_);
    }
    if (publish_tf_)
    {
      btTransform t;

      t.setOrigin(btVector3(x_,y_,0));

      btQuaternion q;
      q.setRPY(0,0, theta_);
      t.setRotation(q);

      tf::StampedTransform transform_msg (
        t, scan_msg->header.stamp, fixed_frame_, base_frame_);
      tf_broadcaster_.sendTransform (transform_msg);
    }
  }
  else
  {
    ROS_WARN("Error in scan matching");
  }

  // **** swap old and new

  ld_free(prev_ldp_scan_);
  prev_ldp_scan_ = curr_ldp_scan;
  last_icp_time_ = new_icp_time;
}

void CanonicalScanMatcher::laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                                LDP& ldp)
{
  unsigned int n = scan_msg->ranges.size();
  ldp = ld_alloc_new(n);

  // derotate
  btTransform base_derot_to_base;
  base_derot_to_base.setOrigin(btVector3(0,0,0));
  btQuaternion imu_oriantation;
  imu_oriantation.setRPY(latest_imu_roll_, latest_imu_pitch_, 0);
  base_derot_to_base.setRotation(imu_oriantation);

  tf::Transform base_derot_to_laser = base_derot_to_base * base_to_laser_;
	
  PointCloudT cloud;

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame

    double r = scan_msg->ranges[i];
    btVector3 p;

    if (r > scan_msg->range_min && r < scan_msg->range_max)
    {
      p.setX(r * a_cos_[i]);
      p.setY(r * a_sin_[i]);
      p.setZ(0.0);

      // transform to derotated base frame and project down

      p = base_derot_to_laser * p;
      p.setZ(0.0);    

      // fill in cloud
/*
      PointT cloud_point;
      cloud_point.x = p.getX();
      cloud_point.y = p.getY();
      cloud_point.z = p.getZ();
      cloud.points.push_back(cloud_point);
*/

      // fill in laser scan data  

      ldp->valid[i] = 1;

      ldp->points[i].p[0] = p.getX();
      ldp->points[i].p[1] = p.getY();

      // these are fake, but csm complains if left empty
      ldp->readings[i] = r;   
      ldp->theta[i]    = scan_msg->angle_min + i * scan_msg->angle_increment; 
    }
    else
    {
      ldp->valid[i] = 0;

      // these are fake, but csm complains if left empty
      ldp->readings[i] = -1;  // for invalid range
      ldp->theta[i]    = scan_msg->angle_min + i * scan_msg->angle_increment;
    }

    ldp->cluster[i]  = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];
 
  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;

  // publish the cloud
/*
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.header= scan_msg->header;
  cloud.header.frame_id = fixed_frame_;
  test_pub_.publish(cloud);
*/
}

void CanonicalScanMatcher::createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  a_cos_.clear(); 
  a_sin_.clear();

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
    a_cos_.push_back(cos(angle));
    a_sin_.push_back(sin(angle));
  }
}

void CanonicalScanMatcher::broadcastTf(const ros::Time& time)
{
//  tf::StampedTransform transform_msg(
//    latest_est_pose_, time, fixed_frame_, base_frame_);

//  tf_broadcaster_.sendTransform (transform_msg);
}    

bool CanonicalScanMatcher::getBaseToLaserTf (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  tf::StampedTransform base_to_laser_tf;
  try
  {
    tf_listener_.waitForTransform(
      base_frame_, scan_msg->header.frame_id, scan_msg->header.stamp, ros::Duration(1.0));
    tf_listener_.lookupTransform (
      base_frame_, scan_msg->header.frame_id, scan_msg->header.stamp, base_to_laser_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("ScanMatcher: Could not get initial laser transform(%s)", ex.what());
    return false;
  }
  base_to_laser_ = base_to_laser_tf;
  laser_to_base_ = base_to_laser_.inverse();

  return true;
}

/*
void CanonicalScanMatcher::laserScanToPCL(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                          PointCloudT::Ptr& cloud_ptr)
{
  cloud_ptr->header = scan_msg->header;

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double r = scan_msg->ranges[i];
    
    if (r > scan_msg->range_min && r < scan_msg->range_max)
    {
      PointT p;
      p.x = r * a_cos_[i];
      p.y = r * a_sin_[i];
      p.z = 0.0;
      cloud_ptr->points.push_back(p);
    }
    else
    {
      cloud_ptr->points.push_back(p_nan_);
    }
  }

  cloud_ptr->width = cloud_ptr->points.size();
  cloud_ptr->height = 1; 
  cloud_ptr->is_dense = false; 
}
*/


} //namespace

