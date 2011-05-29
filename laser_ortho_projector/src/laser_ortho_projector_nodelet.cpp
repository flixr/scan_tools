/*
*  Laser Ortho Projector
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
*/

#include "laser_ortho_projector/laser_ortho_projector_nodelet.h"

typedef scan_tools::LaserOrthoProjectorNodelet LaserOrthoProjectorNodelet;

PLUGINLIB_DECLARE_CLASS (laser_scan_splitter, LaserOrthoProjectorNodelet,
  LaserOrthoProjectorNodelet, nodelet::Nodelet);

void LaserOrthoProjectorNodelet::onInit ()
{
  NODELET_INFO("Initializing LaserOrthoProjector Nodelet");
  
  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  laser_ortho_projector_ = new scan_tools::LaserOrthoProjector(nh, nh_private);
}
