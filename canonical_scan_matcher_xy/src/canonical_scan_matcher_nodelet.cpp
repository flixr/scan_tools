#include "canonical_scan_matcher_xy/canonical_scan_matcher_nodelet.h"

typedef scan_matcher::CanonicalScanMatcherNodelet CanonicalScanMatcherNodelet;

PLUGINLIB_DECLARE_CLASS (canonical_scan_matcher_xy, CanonicalScanMatcherNodelet, 
  CanonicalScanMatcherNodelet, nodelet::Nodelet);

void CanonicalScanMatcherNodelet::onInit ()
{
  NODELET_INFO("Initializing CanonicalScanMatcher Nodelet");
  
  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  scan_matcher_ = new scan_matcher::CanonicalScanMatcher(nh, nh_private);  
}

