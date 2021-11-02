//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

struct Particle {
  Eigen::Vector2f loc;
  float angle;
  double weight;
};

struct Observation {
  std::vector<float>& ranges;
  float range_min;
  float range_max;
  float angle_min;
  float angle_max;
};

class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Possible robot poses
  void MotionModel(Eigen::Vector2f loc, float angle, float dist, float delta_angle);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

  // Succesive Scan Matching Method
  void SLAM::CorrelativeScanMatching(const Observation &new_laser_scan)

 private:

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  // tunable parameters:
  float a1_;
  float a2_;
  float a3_;
  float a4_;

  float min_dist_between_CSM_;
  float min_angle_between_CSM_;

  float num_x_;
  float num_y_;
  float num_angle_;
};
}  // namespace slam

#endif   // SRC_SLAM_H_
