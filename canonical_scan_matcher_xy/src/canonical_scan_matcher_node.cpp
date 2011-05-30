#include "canonical_scan_matcher_xy/canonical_scan_matcher.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "CanonicalScanMatcher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  scan_tools::CanonicalScanMatcher canonical_scan_matcher(nh, nh_private);
  ros::spin();
  return 0;
}
