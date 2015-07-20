/**
 *  This file is part of dvo.
 *
 *  Copyright 2012 Christian Kerl <christian.kerl@in.tum.de> (Technical University of Munich)
 *  For more information see <http://vision.in.tum.de/data/software/dvo>.
 *
 *  dvo is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  dvo is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with dvo.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ECJTU_RGBD_SLAM_NODELET_H_
#define ECJTU_RGBD_SLAM_NODELET_H_

#include <nodelet/nodelet.h>

#include <ecjtu_rgbd_slam/ecjtu_rgbd_slam.h>

namespace ecjtu_rgbd_slam
{

class EcjtuRgbdSLAMNodelet : public nodelet::Nodelet
{
private:
  std::auto_ptr<ecjtu_rgbd_slam::EcjtuRgbdSLAM> tracker_;
public:
  EcjtuRgbdSLAMNodelet();
  virtual ~EcjtuRgbdSLAMNodelet();

  virtual void onInit();
};

} /* namespace ecjtu_rgbd_slam */
#endif /* ECJTU_RGBD_SLAM_NODELET_H_ */
