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

#ifndef CANONICAL_SCAN_MATCHER_CANONICAL_SCAN_MATCHER_NODELET_H
#define CANONICAL_SCAN_MATCHER_CANONICAL_SCAN_MATCHER_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "canonical_scan_matcher/canonical_scan_matcher.h"

namespace scan_tools {

class CanonicalScanMatcherNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit ();

  private:
    CanonicalScanMatcher * scan_matcher_;  // FIXME: change to smart pointer
};

}

#endif // CANONICAL_SCAN_MATCHER_CANONICAL_SCAN_MATCHER_NODELET_H
