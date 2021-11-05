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
\author  Frank Regal & Mary Tebben
\class   cs393r Autonomous Robots
\adapted from:  Joydeep Biswas, (C) 2019
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

// Most Likely Estimated pose
Particle mle_pose_;
Particle give_pose_;

Observation new_scan_;


SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false), 

    // tunable parameters: CSM
    max_particle_cost_(0),
    observation_weight_(1),
    motion_model_weight_(1),
    ray_std_dev_(0.15),

    // tunable parameters: MotionModel
    a1_(0.2), // trans 
    a2_(0.1), // trans 
    a3_(0.4), // angle
    a4_(0.1), // angle   //a's taken from particle_filter.cc

    num_x_(10),     // motion resolution in x
    num_y_(10),     // motion resolution in y
    num_angle_(30),  // motion resolution in angle

    // tunable parameters: ObserveOdometry
    min_dist_between_CSM_(0.5),  // meters
    min_angle_between_CSM_(30*M_PI/180), // radians (30 deg)

    // used for parsing point cloud
    num_ranges_to_skip_(10)
    {}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = give_pose_.loc;
  *angle = give_pose_.angle;
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
  new_scan_.ranges    = ranges;
  new_scan_.range_min = range_min;
  new_scan_.range_max = range_max;
  new_scan_.angle_min = angle_min;
  new_scan_.angle_max = angle_max;

  CorrelativeScanMatching(new_scan_);
}

// Parse the laser scan to a smaller number of ranges
void SLAM::parse_laser_scan(Observation &laser_scan)
{
  int laser_scan_size = laser_scan.ranges.size();

  for(int i = 0; i < laser_scan_size; i++)
  {
    if ((i % num_ranges_to_skip_) != 0)
    {
      laser_scan.ranges.erase(laser_scan.ranges.begin()+i);
    }
  }
}

// Convert Laser Scan to Point Cloud
std::vector<Eigen::Vector2f> SLAM::to_point_cloud(const Observation &laser_scan)
{
  int num_ranges = laser_scan.ranges.size();

  Eigen::Vector2f point(0,0);
  std::vector<Eigen::Vector2f> point_cloud_out;
  float angle_spacing = (laser_scan.angle_max - laser_scan.angle_min) / num_ranges;
  float angle = laser_scan.angle_min;

  for (int i {0}; i < num_ranges; i++)
  {
    point.x() = laser_scan.ranges[i] * cos(angle);
    point.y() = laser_scan.ranges[i] * sin(angle);
    point_cloud_out.push_back(point);
    angle += angle_spacing;
  }

  return point_cloud_out;
}

// Transform Point Cloud to Baselink
void SLAM::TF_to_robot_baselink(Observation &laser_scan)
{
  float delta_angle = (laser_scan.angle_max - laser_scan.angle_min) / laser_scan.ranges.size();
  int laser_scan_size = laser_scan.ranges.size();

  for(int i = 0; i < laser_scan_size; i++){
    float x = laser_scan.ranges[i]*cos(delta_angle*i)+0.2;
    float y = laser_scan.ranges[i]*sin(delta_angle*i);
    laser_scan.ranges[i] = sqrt(pow(x,2) + pow(y,2));
  }
}

// Transform Point Cloud to Last Pose
Eigen::Vector2f SLAM::TF_cloud_to_last_pose(const Eigen::Vector2f cur_points, const Particle &particle)
{
  Vector2f odom_diff = particle.loc - mle_pose_.loc;
  float odom_delta_angle = AngleDiff(particle.angle, mle_pose_.angle);
  Eigen::Rotation2Df R_mle_change(-mle_pose_.angle);
  Eigen::Rotation2Df R_odom_change(odom_delta_angle);
  Vector2f new_point_cloud_last_pose = R_mle_change*odom_diff + R_odom_change*cur_points;
  return new_point_cloud_last_pose;
  }

// Match up Laser Scans and Return the most likely estimated pose (mle_pose_)
void SLAM::CorrelativeScanMatching(Observation &new_laser_scan) 
{
  max_particle_cost_ = 0;

  // parse the incoming laser scan to be more manageable
  parse_laser_scan(new_laser_scan);
  
  // Transfer new_laser_scan to Baselink of Robot
  TF_to_robot_baselink(new_laser_scan);

  // convert to a point cloud  
  std::vector<Eigen::Vector2f> new_point_cloud = to_point_cloud(new_laser_scan);
  
  // Loop through all particles_ from motion model to find best pose
  for (const auto &particle:particles_)
  {
    // cost of the laser scan
    float particle_pose_cost {0};
    float observation_cost {0};

    int point_cloud_size = new_point_cloud.size();

    // transform this laser scan's point cloud to last pose's base_link
    for (int i {0}; i < point_cloud_size; i++)
    {
      Vector2f new_point_cloud_last_pose = TF_cloud_to_last_pose(new_point_cloud[i], particle);
      float x_dist = new_point_cloud_last_pose.x() - last_point_cloud_[i].x();
      float y_dist = new_point_cloud_last_pose.y() - last_point_cloud_[i].y();
      float dist = pow(x_dist,2) + pow(y_dist,2);
      observation_cost += exp(-(pow(dist,2) / pow(ray_std_dev_,2)));
    }
    
    // Calculate the Overall Likelihood of this pose based on weights from the observation and the motion model;
    particle_pose_cost = (observation_cost * observation_weight_) +
                          (particle.weight * motion_model_weight_);
    
    // If this particle is a very high probability, set it as the best guess
    if (particle_pose_cost > max_particle_cost_)
    {
      mle_pose_.angle  = particle.angle;
      mle_pose_.loc    = particle.loc;
      mle_pose_.weight = particle.weight;
      max_particle_cost_ = particle_pose_cost;
    }
  }

  last_point_cloud_.clear();
  last_point_cloud_ = new_point_cloud;

}

void SLAM::MotionModel(Eigen::Vector2f loc, float angle, float dist, float delta_angle){
  particles_.clear();

  // Variance from particle filter motion model
  float variance_x = a1_*dist + a2_*abs(delta_angle);
  float variance_y = a1_*dist + a2_*abs(delta_angle);
  float variance_angle = a3_*dist + a4_*abs(delta_angle);

  // Reference CS393r Lecture Slides "13 - Simultaneous Localization and Mapping" Slides 13 & 14
  // Because we don't know where we are, where we start, which way we are facing
  // all options have to be considered, hence 3D table
  for(int i_x=0; i_x < num_x_; i_x++){
    float deviation_x = variance_x + rng_.Gaussian(0, variance_x);  // Check if correct?
    for(int i_y=0; i_y < num_y_; i_y++){
      float deviation_y = variance_y + rng_.Gaussian(0, variance_y); 
      for(int i_angle=0; i_angle < num_angle_; i_angle++){
        float deviation_angle = variance_angle + rng_.Gaussian(0, variance_angle); 

        float new_location_x = loc.x() + deviation_x*cos(angle) - deviation_y*sin(angle); // + epsilon_x <- added in deviation
        float new_location_y = loc.y() + deviation_x*sin(angle) + deviation_y*cos(angle); // + epsilon_y
        float new_location_angle = angle + deviation_angle; // + epsilon_angle

        float log_weight = -pow(deviation_x/variance_x, 2) - pow(deviation_y/variance_y, 2) - pow(deviation_angle/variance_angle, 2);   // why?

        particles_.push_back({{new_location_x, new_location_y}, new_location_angle, log_weight});
      }
    }
  }
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_){
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }

  // Calculate odom deltas for location and angle
  Vector2f distance = odom_loc - prev_odom_loc_;
  float delta_angle = AngleDiff(odom_angle, prev_odom_angle_);

  // Calculate the magnitude of the distance traveled
  float dist = distance.norm();

  // Calculate the rotation matrix from odom to the most likely estimated pose
  Eigen::Rotation2Df R_odom_to_mle = Eigen::Rotation2Df(mle_pose_.angle - prev_odom_angle_);

  // Update the pose called in GetPose() to return to the simulator
  give_pose_.loc = mle_pose_.loc + R_odom_to_mle*distance;
  give_pose_.angle = ((mle_pose_.angle + delta_angle + M_PI)/(2*M_PI)) - M_PI;


  if(dist > min_dist_between_CSM_ or abs(delta_angle) > min_angle_between_CSM_){
    MotionModel(mle_pose_.loc, mle_pose_.angle, dist, delta_angle);
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
  }
  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.
}

std::vector<Eigen::Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  for(auto point : last_point_cloud_){
    map.push_back(point);
  }
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  for (auto point : last_point_cloud_)
  {
    map.push_back(point);
  }

  return map;
}

}  // namespace slam
