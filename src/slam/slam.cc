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
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "slam.h"

#include "vector_map/vector_map.h"

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

namespace slam {
vector<Particle> particles_;


SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false), 
    
    // tunable parameters:
    a1_(0.2), // trans 
    a2_(0.1), // trans 
    a3_(0.4), // angle
    a4_(0.1), // angle   //a's taken from particle_filter.cc

    dist_between_CSM_(0.5),  // meters
    angle_between_CSM_(30*M_PI/180), // radians

    resolution_x_(10),     // motion resolution in x
    resolution_y_(10),     // motion resolution in y
    resolution_angle_(30)  // motion resolution in angle
    {}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = Vector2f(0, 0);
  *angle = 0;
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
}

void SLAM::MotionModel(Eigen::Vector2f loc, float angle, float dist, float delta_angle){
  particles_.clear();

  // Variance from particle filter motion model
  float variance_x = a1_*dist + a2_*abs(delta_angle);
  float variance_y = a1_*dist + a2_*abs(delta_angle);
  float variance_angle = a3_*dist + a4_*abs(delta_angle);

  // Reference CS393r Lecture Slides "13 - Simultaneous Localization and Mapping" Slides 13 & 14
  for(int i_x=0; i_x < resolution_x_; i_x++){
    float deviation_x = variance_x*(2*i_x/(resolution_x_-1)-1);   // why? find equation
    for(int i_y=0; i_y < resolution_y_; i_y++){
      float deviation_y = variance_y*(2*i_y/(resolution_y_-1)-1); // why?
      for(int i_angle=0; i_angle < resolution_angle_; i_angle++){
        float deviation_angle = variance_angle*(2*i_angle/(resolution_angle_-1)-1); //why?

        float new_location_x = loc.x() + deviation_x*cos(angle) - deviation_y*sin(angle); // + epsilon_x 
        float new_location_y = loc.y() + deviation_x*sin(angle) + deviation_y*cos(angle); // + epsilon_y
        float new_location_angle = angle + deviation_angle; // + epsilon_angle

        float log_weight = -pow(deviation_x/variance_x, 2) - pow(deviation_y/variance_y, 2) - pow(deviation_angle/variance_angle, 2);   // why?

        particles_.push_back({{new_location_x, new_location_y}, new_location_angle, log_weight});
      }
    }
  }
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }

  Vector2f distance = odom_loc - prev_odom_loc_;
  float delta_angle = AngleDiff(odom_angle, prev_odom_angle_);
  float dist = distance.norm();

  if(dist > dist_between_CSM_ or abs(delta_angle) > angle_between_CSM_){
    MotionModel(fill in);
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
  }
  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.
}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}

}  // namespace slam
