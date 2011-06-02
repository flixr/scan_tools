#include "canonical_scan_matcher_xy/canonical_scan_matcher.h"

namespace scan_tools
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
  received_imu_ = 0;
  received_odom_ = 0;

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

  if (use_cloud_input_)
  {
    cloud_subscriber_ = nh_.subscribe(
      cloud_topic_, 1, &CanonicalScanMatcher::cloudCallback, this);
  }
  else
  {
    scan_subscriber_ = nh_.subscribe(
      scan_topic_, 1, &CanonicalScanMatcher::scanCallback, this);
  }

  if (use_imu_)
  {
    imu_subscriber_ = nh_.subscribe(
      imu_topic_, 1, &CanonicalScanMatcher::imuCallback, this);
  }

  if (use_odom_)
  {
    odom_subscriber_ = nh_.subscribe(
      odom_topic_, 1, &CanonicalScanMatcher::imuCallback, this);
  }

  // **** pose publisher
  if (publish_pose_)
  {
    pose_publisher_  = nh_.advertise<geometry_msgs::Pose2D>(
      pose_topic_, 5);
  }
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

  // **** input type - laser scan, or point clouds?
  // if false, will subscrive to LaserScan msgs on /scan. 
  // if true, will subscrive to PointCloud2 msgs on /cloud

  if (!nh_private_.getParam ("use_cloud_input", use_cloud_input_))
    use_cloud_input_= false;

  if (use_cloud_input_)
  {
    if (!nh_private_.getParam ("min_cloud_angle_", min_cloud_angle_))
      min_cloud_angle_ = -M_PI/2.0;
    if (!nh_private_.getParam ("max_cloud_angle_", max_cloud_angle_))
      max_cloud_angle_ = M_PI/2.0;
  }

  // **** What predictions are available to speed up the ICP?
  // 1) imu - [theta] from imu yaw angle - /odom topic
  // 2) odom - [x, y, theta] from wheel odometry - /imu topic
  // 3) alpha_beta - [x, y, theta] from simple tracking filter - no topic req.
  // If more than one is enabled, priority is imu > odom > alpha_beta

  if (!nh_private_.getParam ("use_imu", use_imu_))
    use_imu_ = true;
  if (!nh_private_.getParam ("use_odom", use_odom_))
    use_odom_ = true;
  if (!nh_private_.getParam ("use_alpha_beta", use_alpha_beta_))
    use_alpha_beta_ = true;

  // **** How to publish the output?
  // tf (fixed_frame->base_frame), 
  // pose message (pose of base frame in the fixed frame)

  if (!nh_private_.getParam ("publish_tf", publish_tf_))
    publish_tf_ = true;
  if (!nh_private_.getParam ("publish_pose", publish_pose_))
    publish_pose_ = true;

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
}

void CanonicalScanMatcher::imuCallback (const sensor_msgs::ImuPtr& imu_msg)
{
  btQuaternion q;
	tf::quaternionMsgToTF(imu_msg->orientation, q);
  btMatrix3x3 m(q);
  double temp;
  boost::mutex::scoped_lock(mutex_);
  received_imu_++;
  m.getRPY(temp, temp, latest_imu_yaw_);
}

void CanonicalScanMatcher::odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  boost::mutex::scoped_lock(mutex_);
  received_odom_++;
  latest_odom_ = *odom_msg;
}

void CanonicalScanMatcher::cloudCallback (const PointCloudT::ConstPtr& cloud)
{
  // **** if first scan, cache the tf from base to the scanner

  if (!initialized_)
  {
    // cache the static tf from base to laser
    if (!getBaseToLaserTf(cloud->header.frame_id))
    {
      ROS_WARN("ScanMatcher: Skipping scan");
      return;
    }

    PointCloudToLDP(cloud, prev_ldp_scan_);
    last_icp_time_ = cloud->header.stamp;
    last_imu_yaw_ = latest_imu_yaw_;
    last_odom_ = latest_odom_;
    initialized_ = true;
  }

  LDP curr_ldp_scan;
  PointCloudToLDP(cloud, curr_ldp_scan);
  processScan(curr_ldp_scan, cloud->header.stamp);
}

void CanonicalScanMatcher::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  // **** if first scan, cache the tf from base to the scanner
  
  if (!initialized_)
  {
    createCache(scan_msg);    // caches the sin and cos of all angles

    // cache the static tf from base to laser
    if (!getBaseToLaserTf(scan_msg->header.frame_id))
    {
      ROS_WARN("ScanMatcher: Skipping scan");
      return;
    }

    laserScanToLDP(scan_msg, prev_ldp_scan_); 
    last_icp_time_ = scan_msg->header.stamp;
    last_imu_yaw_ = latest_imu_yaw_;
    last_odom_ = latest_odom_;
    initialized_ = true;
  }

  LDP curr_ldp_scan;
  laserScanToLDP(scan_msg, curr_ldp_scan);
  processScan(curr_ldp_scan, scan_msg->header.stamp);
}

void CanonicalScanMatcher::processScan(LDP& curr_ldp_scan, const ros::Time& time)
{
  struct timeval start_, end_;    // used for timing
  gettimeofday(&start_, NULL);

  // CSM is used in the following way:
  // The reference scan (prevLDPcan_) always has a pose of 0
  // The new scan (currLDPScan) has a pose equal to the movement
  // of the laser in the world frame since the last scan (btTransform change)
  // The computed correction is then propagated using the tf machinery

  prev_ldp_scan_->odometry[0] = 0;
  prev_ldp_scan_->odometry[1] = 0;
  prev_ldp_scan_->odometry[2] = 0;

  prev_ldp_scan_->estimate[0] = 0;
  prev_ldp_scan_->estimate[1] = 0;
  prev_ldp_scan_->estimate[2] = 0;

  prev_ldp_scan_->true_pose[0] = 0;
  prev_ldp_scan_->true_pose[1] = 0;
  prev_ldp_scan_->true_pose[2] = 0;

  input_.laser_ref  = prev_ldp_scan_;
  input_.laser_sens = curr_ldp_scan;

  // **** estimated change since last scan

  ros::Time new_icp_time = ros::Time::now();
  ros::Duration dur = new_icp_time - last_icp_time_;
  double dt = dur.toSec();

  double pr_ch_x, pr_ch_y, pr_ch_a;
  getPrediction(pr_ch_x, pr_ch_y, pr_ch_a, dt);

  input_.first_guess[0] = ( cos_theta_ * pr_ch_x + sin_theta_ * pr_ch_y);
  input_.first_guess[1] = (-sin_theta_ * pr_ch_x + cos_theta_ * pr_ch_y);
  input_.first_guess[2] = pr_ch_a;

  // *** scan match - using icp (xy means x and y are already computed)

  sm_icp_xy(&input_, &output_);

  if (output_.valid) 
  {
    // **** calculate change in position of the laser

    double dx = (cos_theta_ * output_.x[0] - sin_theta_ * output_.x[1]);
    double dy = (sin_theta_ * output_.x[0] + cos_theta_ * output_.x[1]);
    double da = output_.x[2];

    if(use_alpha_beta_)
    {
      double r_x = dx - pr_ch_x;
      double r_y = dy - pr_ch_y;
      double r_a = da - pr_ch_a;

      x_     = (x_     + pr_ch_x) + alpha_ * r_x;
      y_     = (y_     + pr_ch_y) + alpha_ * r_y;
      theta_ = (theta_ + pr_ch_a) + alpha_ * r_a;

      v_x_     = v_x_     + (beta_ / dt) * r_x;
      v_y_     = v_y_     + (beta_ / dt) * r_y;
      v_theta_ = v_theta_ + (beta_ / dt) * r_a;
    }
    else
    {
      x_     += dx;
      y_     += dy;
      theta_ += da;
    }

    cos_theta_ = cos(theta_);
    sin_theta_ = sin(theta_);

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

      tf::StampedTransform transform_msg (t, time, fixed_frame_, base_frame_);
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

  // **** statistics

  gettimeofday(&end_, NULL);
  double icp_dur = ((end_.tv_sec   * 1000000 + end_.tv_usec  ) -
                    (start_.tv_sec * 1000000 + start_.tv_usec)) / 1000.0;
  ROS_DEBUG("scan matcher ICP duration: %.1f ms \n", icp_dur);
}

void CanonicalScanMatcher::PointCloudToLDP(const PointCloudT::ConstPtr& cloud,
                                                 LDP& ldp)
{
  unsigned int n = cloud->points.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++)
  {
    // calculate position in laser frame

    if (!is_nan(cloud->points[i].z))
    {
      // fill in laser scan data

      ldp->valid[i] = 1;

      ldp->points[i].p[0] = cloud->points[i].x;
      ldp->points[i].p[1] = cloud->points[i].y;

      // these are fake, but csm complains if left empty
      ldp->readings[i] = 1.0;
    }
    else
    {
      ldp->valid[i] = 0;

      // these are fake, but csm complains if left empty
      ldp->readings[i] = -1;  // for invalid range
    }

    // these are fake, but csm complains if left empty
    ldp->theta[i] = max_cloud_angle_ + 
                   (double)n/(double)i * (max_cloud_angle_ - min_cloud_angle_);

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
}

void CanonicalScanMatcher::laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                                LDP& ldp)
{
  unsigned int n = scan_msg->ranges.size();
  ldp = ld_alloc_new(n);

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

      // fill in laser scan data  

      ldp->valid[i] = 1;

      ldp->points[i].p[0] = p.getX();
      ldp->points[i].p[1] = p.getY();

      // these are not needed, but csm complains if left empty
      ldp->readings[i] = r;   
      ldp->theta[i]    = scan_msg->angle_min + i * scan_msg->angle_increment; 
    }
    else
    {
      ldp->valid[i] = 0;

      // these are not needed, but csm complains if left empty
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

bool CanonicalScanMatcher::getBaseToLaserTf (const std::string& frame_id)
{
  tf::StampedTransform base_to_laser_tf;
  try
  {
    tf_listener_.waitForTransform(
      base_frame_, frame_id, ros::Time::now(), ros::Duration(1.0));
    tf_listener_.lookupTransform (
      base_frame_, frame_id, ros::Time::now(), base_to_laser_tf);
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

// returns the predicted change in pose (in fixed frame)
// since the last time we did icp
void CanonicalScanMatcher::getPrediction(double& pr_ch_x, double& pr_ch_y, 
                                         double& pr_ch_a, double dt)
{
  // **** base case - no input available, use zero-motion model
  pr_ch_x = 0.0;
  pr_ch_y = 0.0;
  pr_ch_a = 0.0;

  // **** use alpha-beta tracking (const. vel. model)
  if (use_alpha_beta_)
  {
    //@FIXME: lock

    // estmate change in fixed frame, using fixed velocity
    pr_ch_x = v_x_     * dt;     // in fixed frame
    pr_ch_y = v_y_     * dt;
    pr_ch_a = v_theta_ * dt;
  }

  // **** use wheel odometry
  if (use_odom_ && received_odom_ > 1)
  {
    //@FIXME: mutex
    pr_ch_x = latest_odom_.pose.pose.position.x - 
              last_odom_.pose.pose.position.x;

    pr_ch_y = latest_odom_.pose.pose.position.y - 
              last_odom_.pose.pose.position.y;

    pr_ch_a = getYawFromQuaternion(latest_odom_.pose.pose.orientation) -
              getYawFromQuaternion(last_odom_.pose.pose.orientation);

    last_odom_ = latest_odom_;
  }

  // **** use imu
  if (use_imu_ && received_imu_ > 1)
  {
    //@FIXME: mutex
    pr_ch_a = latest_imu_yaw_ - last_imu_yaw_;
    last_imu_yaw_ = latest_imu_yaw_;
  }
}

double CanonicalScanMatcher::getYawFromQuaternion(
  const geometry_msgs::Quaternion& quaternion)
{
  double temp, yaw;
  btQuaternion q;
  tf::quaternionMsgToTF(quaternion, q);
  btMatrix3x3 m(q);
  m.getRPY(temp, temp, yaw);
  return yaw;
}

} //namespace

