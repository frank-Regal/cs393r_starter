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
Particle mle_pose_;        // best pose updated every laser scan
Particle give_pose_;       // output pose 
Observation new_scan_;     // holder for new laser scan observed
Observation initial_scan_; // initial scan
LookupTable table_;        // global lookup table

SLAM::SLAM() :
  prev_odom_loc_(0, 0),
  prev_odom_angle_(0),
  odom_initialized_(false), 

  // tunable parameters: CSM
  max_particle_cost_(0),
  observation_weight_(3),
  motion_model_weight_(1),
  ray_std_dev_(0.01),

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
  num_ranges_to_skip_(5),
  update_scan_(false),
  first_scan_(false),
  table_check_(false)
  {
    InitializeLookupTable();
  }

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = give_pose_.loc;
  *angle = give_pose_.angle;
}

std::vector<Eigen::Vector2f> SLAM::GetMap() {
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  //if (first_scan_ == true and odom_initialized_ == true)
  //  map.clear();
  
  return map;
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_){
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    //mle_pose_ = Particle({Vector2f(0,0),0,0});
    odom_initialized_ = true;
    update_scan_ = true; // check_ was false worked alittle better with true
    first_scan_ = true;
    return;
  }

  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.

  // Calculate odom deltas for location and angle
  Vector2f distance = odom_loc - prev_odom_loc_;
  float delta_angle = AngleDiff(odom_angle, prev_odom_angle_);

  // Calculate the magnitude of the distance traveled
  float dist = distance.norm();

  // Update the pose called in GetPose() to return to the simulator
  give_pose_.loc = mle_pose_.loc + R_odom_to_mle*distance;
  give_pose_.angle = fmod(mle_pose_.angle + delta_angle + M_PI,2*M_PI) - M_PI;
  
  if(dist > min_dist_between_CSM_ or abs(delta_angle) > min_angle_between_CSM_){
    MotionModel(give_pose_.loc, give_pose_.angle, dist, delta_angle);
    update_scan_ = true;
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
  }
  
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
  /*
  if (first_scan_ == true and odom_initialized_ == true)
  {
    initial_scan_.ranges    = TrimRanges(ranges,range_min,range_max);
    initial_scan_.range_min = range_min;
    initial_scan_.range_max = range_max;
    initial_scan_.angle_min = angle_min;
    initial_scan_.angle_max = angle_max;

    initial_scan_ = parse_laser_scan(initial_scan_);

    TF_to_robot_baselink(initial_scan_);
    last_point_cloud_ = to_point_cloud(initial_scan_);
    for (auto point : last_point_cloud_)
      map.push_back(point);
    first_scan_ = false;
  }
  */

  //return;

  if (update_scan_ == true or (odom_initialized_ == true and not table_check_))
  {
    new_scan_.ranges    = ranges; //TrimRanges(ranges,range_min,range_max);
    new_scan_.range_min = range_min;
    new_scan_.range_max = range_max;
    new_scan_.angle_min = angle_min;
    new_scan_.angle_max = angle_max;

    mle_pose_ = CorrelativeScanMatching(new_scan_);
    CombineMap(mle_pose_);

    // Calculate the rotation matrix from odom to the most likely estimated pose
    R_odom_to_mle = Eigen::Rotation2Df(mle_pose_.angle - prev_odom_angle_);

    ResetLookupTable();
    for(auto point : last_point_cloud_){
      ApplyGuassianBlur(point);
    }
    table_check_ = true;
    update_scan_ = false;
  }
}

void SLAM::InitializeLookupTable(){
  table_.start_loc.x() = -8;
  table_.start_loc.y() = -8;
  table_.min_cost = -1000; 
  table_.overall_width = 16;
  table_.overall_height = 16;
  table_.cell_resolution = 0.02;
  table_.cell_width = table_.overall_width/table_.cell_resolution;
  table_.cell_height = table_.overall_height/table_.cell_resolution;
  ResetLookupTable();
}

void SLAM::ResetLookupTable(){
  inner_vec.clear();
  cell.clear();
  for (int j {0}; j < table_.cell_width; j++){
    for (int k {0}; k < table_.cell_height; k++){
      inner_vec.push_back(table_.min_cost);
    }
    cell.push_back(inner_vec);
  }
}

Eigen::Vector2f SLAM::GetCellIndex(const Eigen::Vector2f loc) 
{
  Eigen::Vector2f xy_diff = loc - table_.start_loc;
  int x_index = xy_diff.x() / table_.cell_resolution;
  int y_index = xy_diff.y() / table_.cell_resolution;
  return Eigen::Vector2f (x_index, y_index);
}

std::vector<float> SLAM::TrimRanges(const vector<float> &ranges, const float range_min, const float range_max)
{
  vector<float> trimmed_scan;

  for (auto line : ranges)
    {
      if (line < range_max * 0.95 and line > range_min * 1.05)
      {
        trimmed_scan.push_back(line);
      }else{
        trimmed_scan.push_back(0);
      }
    }

  return trimmed_scan;
}

Observation SLAM::parse_laser_scan(const Observation &laser_scan)
{ // Parse the laser scan to a smaller number of ranges
  //return laser_scan;
  int laser_scan_size = laser_scan.ranges.size();
  Observation parsed_laser_scan;
  parsed_laser_scan.range_min = laser_scan.range_min;
  parsed_laser_scan.range_max = laser_scan.range_max;
  parsed_laser_scan.angle_min = laser_scan.angle_min;
  parsed_laser_scan.angle_max = laser_scan.angle_max;
  
  for(int i = 0; i < laser_scan_size; i++)
  {
    if ((i % num_ranges_to_skip_) == 0)
      parsed_laser_scan.ranges.push_back(laser_scan.ranges[i]);
  }
  return parsed_laser_scan;
}

std::vector<Eigen::Vector2f> SLAM::to_point_cloud(const Observation &laser_scan)
{ // Convert Laser Scan to Point Cloud
  //return std::vector<Eigen::Vector2f> {Vector2f(0,0)};
  int num_ranges = laser_scan.ranges.size();

  Eigen::Vector2f point(0,0);
  std::vector<Eigen::Vector2f> point_cloud_out;
  float angle_spacing = (laser_scan.angle_max - laser_scan.angle_min) / num_ranges;
  float angle = laser_scan.angle_min;

  for (int i {0}; i < num_ranges; i++)
  {
    if(laser_scan.ranges[i] == 0){
      continue;
    }else{
    point.x() = laser_scan.ranges[i] * cos(angle);
    point.y() = laser_scan.ranges[i] * sin(angle);
    point_cloud_out.push_back(point);
    angle += angle_spacing;
    }
  }

  return point_cloud_out;
}

void SLAM::TF_to_robot_baselink(Observation &laser_scan) // checked
{ // Transform LaserScan to Baselink
  //return;
  float delta_angle = (laser_scan.angle_max - laser_scan.angle_min) / laser_scan.ranges.size();
  int laser_scan_size = laser_scan.ranges.size();

  float angle_iter = laser_scan.angle_min;
  for(int i = 0; i < laser_scan_size; i++){
    float x = laser_scan.ranges[i]*cos(angle_iter)-0.2;
    float y = laser_scan.ranges[i]*sin(angle_iter);
    laser_scan.ranges[i] = sqrt(pow(x,2) + pow(y,2));
    angle_iter += delta_angle;
  }
  
}

Eigen::Vector2f SLAM::TF_cloud_to_last_pose(const Eigen::Vector2f cur_points, const Particle &particle)
{ // Transform Point Cloud to Last Pose
  // return Vector2f(0,0);
  
  // Calculate Translational and Rotational Deltas
  Vector2f odom_diff = particle.loc - mle_pose_.loc;
  float odom_delta_angle = AngleDiff(particle.angle, mle_pose_.angle);

  // Calculate Rotation Matrices
  Eigen::Rotation2Df R_mle_change(-mle_pose_.angle);
  Eigen::Rotation2Df R_odom_change(odom_delta_angle);

  // Transform Point Cloud Points to most likely estimated pose from last step
  Vector2f new_point_cloud_last_pose = R_mle_change*odom_diff + R_odom_change*cur_points;

  return new_point_cloud_last_pose;
}

bool SLAM::InCellBounds(int x, int y){
  bool check_x = (x >= 0 and x < table_.cell_width);
  bool check_y = (y >= 0 and y < table_.cell_height);
  if(check_x and check_y)
    return(true);
  else
    return(false);
}

void SLAM::ApplyGuassianBlur(const Eigen::Vector2f point)
{ // Fills the lookup table with log likelihoods for each cell within the table
  Eigen::Vector2f i = GetCellIndex(point);
  
  float xi {0};
  float yi {0};
  float magnitude_from_point {0};
  float log_likelihood {0};
  
  // Loop through all x- & y-cells
  int j {0}; 
  bool log_likelihood_x {true};
  while (log_likelihood_x == true)
  { 
    int k {0};
    while (log_likelihood > table_.min_cost)
    {
      // Calculate Log Likelihoods 
      xi = table_.cell_resolution*j;
      yi = table_.cell_resolution*k;
      magnitude_from_point = pow(xi,2) + pow(yi,2);
      log_likelihood = -magnitude_from_point / pow(ray_std_dev_,2);
      
      // Updated Each Grid Cell with Log-Likelihood Weight
      if (InCellBounds(i.x()+xi, i.y()+yi))
        cell[i.x()+xi][i.y()+yi] = std::max(cell[i.x()+xi][i.y()+yi],log_likelihood);
      if (InCellBounds(i.x()+xi, i.y()-yi))
        cell[i.x()+xi][i.y()-yi] = std::max(cell[i.x()+xi][i.y()-yi],log_likelihood);
      if (InCellBounds(i.x()-xi, i.y()+yi))
        cell[i.x()-xi][i.y()+yi] = std::max(cell[i.x()-xi][i.y()+yi],log_likelihood);
      if (InCellBounds(i.x()-xi, i.y()-yi))
        cell[i.x()-xi][i.y()-yi] = std::max(cell[i.x()-xi][i.y()-yi],log_likelihood);
    
      // Increase Iter
      k++;      
    }
    j++;
    log_likelihood_x = -(pow(j*table_.cell_resolution, 2) / pow(ray_std_dev_,2)) > table_.min_cost; 
  }  
    

}

void SLAM::CombineMap(const Particle pose)
{
  int num_ranges = new_scan_.ranges.size();
  Eigen::Vector2f point(0,0);
  float angle_spacing = (new_scan_.angle_max - new_scan_.angle_min) / num_ranges;
  float angle = new_scan_.angle_min;

  for (int i {0}; i < num_ranges; i++)
  {
    point.x() = pose.loc.x() + new_scan_.ranges[i]*cos(pose.angle + angle);
    point.y() = pose.loc.y() + new_scan_.ranges[i]*sin(pose.angle + angle);
    angle += angle_spacing;
    map.push_back(point);
  }
}

void SLAM::MotionModel(Eigen::Vector2f loc, float angle, float dist, float delta_angle){
  //particles_.push_back({{1,1},0,0});
  //return;
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

Particle SLAM::CorrelativeScanMatching(const Observation &new_laser_scan) 
{ // Match up Laser Scans and Return the most likely estimated pose (mle_pose_)
  //return mle_pose_;
  max_particle_cost_ = 0;
  mle_pose_ = {{0,0},0,0};

  // parse the incoming laser scan to be more manageable
  Observation parsed_laser_scan = parse_laser_scan(new_laser_scan);
  
  // Transfer new_laser_scan to Baselink of Robot
  TF_to_robot_baselink(parsed_laser_scan);

  // convert to a point cloud  
  std::vector<Eigen::Vector2f> new_point_cloud = to_point_cloud(parsed_laser_scan);
  
  int point_cloud_size = new_point_cloud.size();
  //int last_point_cloud_size = last_point_cloud_.size();

  // Loop through all particles_ from motion model to find best pose
  for (const auto &particle:particles_)
  {
    // cost of the laser scan
    float particle_pose_cost {0};
    float observation_cost {0};

    // transform this laser scan's point cloud to last pose's base_link
    for (int i {0}; i < point_cloud_size; i++)
    {
      Eigen::Vector2f new_point_cloud_last_pose = TF_cloud_to_last_pose(new_point_cloud[i], particle);
      Eigen::Vector2f new_cost_index = GetCellIndex(new_point_cloud_last_pose);
      
      if(InCellBounds(new_cost_index.x(), new_cost_index.y()))
      {
        observation_cost += cell[new_cost_index.x()][new_cost_index.y()];
      }
      else 
        continue;
    }
    
    //float norm_observation_cost = observation_cost/point_cloud_size;
    //float norm_motion_model_cost = particle.weight/3;
    
    //std::cout << "Observation Cost: " << observation_cost << std::endl;
    // Calculate the Overall Likelihood of this pose based on weights from the observation and the motion model;
    particle_pose_cost = (observation_cost * observation_weight_) +
                          (particle.weight * motion_model_weight_);
    //std::cout << "particle_pose_cost: " << particle_pose_cost << std::endl;
    // If this particle is a very high probability, set it as the best guess
    if (particle_pose_cost < max_particle_cost_)
    {
      mle_pose_.loc    = particle.loc;
      max_particle_cost_ = particle_pose_cost;
    }
  }
  last_point_cloud_ = new_point_cloud;
  std::cout << "updated last_point_cloud" << std::endl;
  return mle_pose_;
}


}  // namespace slam
