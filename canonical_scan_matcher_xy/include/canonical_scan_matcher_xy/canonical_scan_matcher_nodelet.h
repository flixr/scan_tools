#ifndef CANONICAL_SCAN_MATCHER_XY_CANONICAL_SCAN_MATCHER_NODELET_H
#define CANONICAL_SCAN_MATCHER_XY_CANONICAL_SCAN_MATCHER_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "canonical_scan_matcher_xy/canonical_scan_matcher.h"

namespace scan_matcher {

class CanonicalScanMatcherNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit ();

  private:
    CanonicalScanMatcher * scan_matcher_;  // FIXME: change to smart pointer
};

}

#endif // CANONICAL_SCAN_MATCHER_XY_CANONICAL_SCAN_MATCHER_NODELET_H
