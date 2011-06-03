/*
*  Canonical Scan Matcher
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
*
*  This is a wrapper around Canonical Scan Matcher [1], written by 
*  Andrea Censi
*
*  [1] A. Censi, "An ICP variant using a point-to-line metric" 
*  Proceedings of the IEEE International Conference 
*  on Robotics and Automation (ICRA), 2008
*/

#include "canonical_scan_matcher/canonical_scan_matcher_nodelet.h"

typedef scan_tools::CanonicalScanMatcherNodelet CanonicalScanMatcherNodelet;

PLUGINLIB_DECLARE_CLASS (canonical_scan_matcher_xy, CanonicalScanMatcherNodelet, 
  CanonicalScanMatcherNodelet, nodelet::Nodelet);

void CanonicalScanMatcherNodelet::onInit ()
{
  NODELET_INFO("Initializing CanonicalScanMatcher Nodelet");
  
  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  scan_matcher_ = new scan_tools::CanonicalScanMatcher(nh, nh_private);
}

