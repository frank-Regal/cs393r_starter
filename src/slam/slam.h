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
#include "shared/util/random.h"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

struct Particle {
  Eigen::Vector2f loc;
  float angle;
  double weight;
};

struct Observation {
  // Struct to Hold Observation from Laser Scan
  std::vector<float> ranges;
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

  // Successive Scan Matching Method (Refrence: Olson, 2009)
  Particle CorrelativeScanMatching(Observation &new_laser_scan);

  // Parse the laser scan to a smaller number of ranges
  Observation parse_laser_scan(const Observation &laser_scan);

  // Convert Laser Scan to Point Cloud
  std::vector<Eigen::Vector2f> to_point_cloud(const Observation &laser_scan);

  // Transform Point Cloud to Baselink
  void TF_to_robot_baselink(Observation &laser_scan);

  // Transform Point Cloud to Last Pose
  Eigen::Vector2f TF_cloud_to_last_pose(const Eigen::Vector2f cur_points, const Particle &particle);

  // Combine the Map
  void CombineMap(const Particle pose);

 private:

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  // Random number generator.
  util_random::Random rng_;

  // tunable parameters: CSM
  float max_particle_cost_;
  float observation_weight_;
  float motion_model_weight_;

  // Standard deviation of Physical LIDAR System; set_parameter
  double ray_std_dev_;

  // tunable parameters: MotionModel
  float a1_; 
  float a2_; 
  float a3_;;
  float a4_;

  float num_x_;
  float num_y_;
  float num_angle_;

  // tunable parameters: ObserveOdometry
  float min_dist_between_CSM_;
  float min_angle_between_CSM_;

  int num_ranges_to_skip_;

  std::vector<Eigen::Vector2f> last_point_cloud_;
  std::vector<Eigen::Vector2f> initial_point_cloud_;

  bool update_scan_;
  bool first_scan_;

  Eigen::Rotation2Df R_odom_to_mle;

  std::vector<Eigen::Vector2f> map;

};
}  // namespace slam

#endif   // SRC_SLAM_H_
