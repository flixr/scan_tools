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

#ifndef LASER_ORTHO_PROJECTOR_LASER_ORTHO_PROJECTOR_NODELET_H
#define LASER_ORTHO_PROJECTOR_LASER_ORTHO_PROJECTOR_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "laser_ortho_projector/laser_ortho_projector.h"

namespace scan_tools {

class LaserOrthoProjectorNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    LaserOrthoProjector * laser_ortho_projector_;  // FIXME: change to smart pointer
};

} //namespace scan tools

#endif //  LASER_ORTHO_PROJECTOR_LASER_ORTHO_PROJECTOR_NODELET_H
