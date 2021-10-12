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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
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
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

#define _USE_MATH_DEFINES
#include <math.h>

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");


namespace {
  int predict_steps {0};
  double max_particle_weight {0};
  int updates_done {0};
}

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

// helper function
double ParticleFilter::get_angle_diff(double a, double b)
{
  double theta = a-b;
  while (theta < -M_PI)
  {
    theta += 2*M_PI;
  }
  while (theta > M_PI)
  {
    theta -= 2*M_PI;
  }

  return theta;
}


ParticleFilter::ParticleFilter() :
    odom_old(0,0,0),
    prev_odom_loc_(0, 0), // not needed CHECK
    prev_odom_angle_(0),  // not needed CHECK
    odom_initialized_(false),
    predict_step_done_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) 
{
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Setting Up Output Vector
  vector<Vector2f>& scan = *scan_ptr;

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  
  // Initialize Max Distance Point Vector
  Eigen::Vector2f endpoint_of_max_distance_ray;

  // Step Size of Scan; set_parameter
  int step_size_of_scan {20};

  // Reduce the input vectors size to account for every 10th laser scan ray
  int length_of_scan_vec = num_ranges/step_size_of_scan;

  //std::cout << "length_of_scan_vec: " << length_of_scan_vec << std::endl; //debug
  scan.resize(length_of_scan_vec);

  // Calcs: Predicted Beginning and End Points of Each Ray within Theoretical Laser Scan
  // Points for Laser Scanner in Space using Distance between baselink and laser scanner on physical robot to be 0.2 meters
  float laser_scanner_loc_x = loc.x() + 0.2*cos(angle);
  float laser_scanner_loc_y = loc.y() + 0.2*sin(angle);
  Eigen::Vector2f laser_scanner_loc(laser_scanner_loc_x,laser_scanner_loc_y);

  // Starting Angle based on the orientation of the particle (angle)
  float parsed_angle = angle+angle_min;

  // Build Vector of Laser Scan Angles
  std::vector <float> predicted_scan_angles;

  // Fill a Vector of each Scan Angle
  for (int i {0}; i < length_of_scan_vec; i++)
  {
    parsed_angle += ((angle_max-angle_min)/length_of_scan_vec);
    predicted_scan_angles.push_back(parsed_angle);
  }

  // loop through each theoretical scan
  for (int j {0}; j < length_of_scan_vec; j++)
  {
    // Initialize the points of the predicted laser scan rays
    line2f laser_ray(1,2,3,4);
    laser_ray.p0.x() = laser_scanner_loc_x + range_min*cos(predicted_scan_angles[j]);
    laser_ray.p0.y() = laser_scanner_loc_y + range_min*sin(predicted_scan_angles[j]);
    laser_ray.p1.x() = laser_scanner_loc_x + range_max*cos(predicted_scan_angles[j]);
    laser_ray.p1.y() = laser_scanner_loc_y + range_max*sin(predicted_scan_angles[j]);
    
    // Fill i-th entry to return vector (scan) with each point of predicted laser scan of the max range
    scan[j] << laser_ray.p1.x(),
               laser_ray.p1.y();
    
    // Calculate the max distance of the current ray
    double max_distance_of_current_ray = (laser_scanner_loc - scan[j]).norm();

    // Loop Through Each Line from Imported Map Text File to See this Single Laser Ray
    // Intersects at the angle of predicted_scan_angles
    for (size_t k = 0; k < map_.lines.size(); ++k) 
    {
      // Assign Map Lines to Variable for Interestion Calculations
      const line2f map_line = map_.lines[k];
      
      // Initialize Return Variable of the Location where the Ray Intersects
      Eigen::Vector2f intersection_point;

      // Compare Map Text File Lines to Theoretical Laser Rays from Each Particle
      bool intersects = map_line.Intersection(laser_ray, &intersection_point);
      
      if (intersects) // is true
      {
        // Record Distance of that Intersecting Ray
        double distance_of_intersecting_ray = (intersection_point-laser_scanner_loc).norm();
        // If the Intersecting Distance Recorded above is less than of all of the previously recorded intersections for this ray at specific angele
        if ( distance_of_intersecting_ray < max_distance_of_current_ray)
        {
          // Set Distance Variable for Next Comparison
          max_distance_of_current_ray = distance_of_intersecting_ray;
          // Record the Endpoint of Where that Ray Intersects
          endpoint_of_max_distance_ray = intersection_point;
        }
      }
    }
    // Fill Output Vector of this Ray with the point at where the shortest distance ray intersects with the map
    scan[j] = endpoint_of_max_distance_ray; 
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.

  // Setting Up Output Variable
  Particle& particle = *p_ptr;

  // Initialize point cloud vector to fill from GetPredictedPointCloud
  vector<Vector2f> predicted_point_cloud;

  // Fill point cloud vector
  GetPredictedPointCloud(particle.loc, particle.angle, 
                         ranges.size(), range_min, range_max, 
                         angle_min, angle_max, &predicted_point_cloud);

  // Resize lidar scan range (Number of Rays)
  vector<float> resized_ranges(predicted_point_cloud.size());
  int lidar_ray_step_size = ranges.size() / predicted_point_cloud.size();

  // Calculating the Size of Predicted Point Cloud Length
  int predicted_point_cloud_length = predicted_point_cloud.size();

  // Resizing the Actual Laser Scan Ranges
  for(int i = 0; i < predicted_point_cloud_length; i++)
    resized_ranges[i] = ranges[lidar_ray_step_size*i];

  // Tuning parameters for the minimum and maximum distances of the laser scanner; set_parameter
  double dshort = 0.5;
  double dlong = 0.5;
  double gamma = 0.8;

  // // Adding tuning params for Min and Max Ranges of Physical Laser Scanner
  // float min_range_with_tuning = range_min + range_min * min_dist_tuning;
  // float max_range_with_tuning = range_max - range_max * max_dist_tuning;

  // Standard deviation of Physical LIDAR System; set_parameter
  double ray_std_dev = 0.1;

  // Reset variables between each point cloud
  float weight {0};
  float total_weight {0};

  // Calculate Weight for Particle
  for(int i = 0; i < predicted_point_cloud_length; i++)
  {
    //std::cout << "\n\n[Predicted Point Cloud: " << i << "]" << std::endl; //debug
    // Physical Laser Scanner Location is Offset From the Particle Location
    float laser_scanner_loc_x = particle.loc.x() + 0.2*cos(particle.angle);
    float laser_scanner_loc_y = particle.loc.y() + 0.2*sin(particle.angle);

    // Distance laser scanner found to be from obstacle (actual distance)
    float particle_actual_distance = resized_ranges[i];

    // Calculate distance between the laser scanner and the returned point cloud location (theoretical distance)
    float theoretical_dist_x = predicted_point_cloud[i].x() - laser_scanner_loc_x;
    float theoretical_dist_y = predicted_point_cloud[i].y() - laser_scanner_loc_y;
    float particle_theoretical_distance = sqrt(pow(theoretical_dist_x, 2)+pow(theoretical_dist_y, 2));

    // Calculating Distance Comparison
    float delta_distance = particle_actual_distance - particle_theoretical_distance;
    //std::cout << "\nDelta Distance: " << delta_distance << std::endl; //debug

    // Function for weighting the probability of particle being in the location found
    if(particle_actual_distance < range_min or particle_actual_distance > range_max)
      continue;
    else if (particle_actual_distance < (particle_theoretical_distance - dshort))
      weight = exp(-(pow(dshort,2) / pow(ray_std_dev,2)));
    else if (particle_actual_distance > (particle_theoretical_distance + dlong))
      weight = exp(-(pow(dlong,2) / pow(ray_std_dev,2)));
    else
    {
      weight = exp(-(pow(delta_distance,2) / pow(ray_std_dev,2)));
      //std::cout << "****Actual is in the Guassian" << std::endl; // debug
    }
    // Update Total Probability for this particle
    //std::cout << "weight: " << weight << std::endl; //debug
    total_weight += weight;
  }
  particle.weight = gamma * total_weight;
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 

  // Predefine the Number of Resamples; set_parameter
  int num_of_resamples {50};

  // Initializations
  std::vector <Particle> reduced_particle_vec; // return vector (vector of the kept particles)
  double weight_sum {0};                       // comparison variable 
  double total_weight {0};                     // total weight of all particles
  int k {0};                                   // edge case variable
  int input_vec_length = particles_.size();    // Get Length of Input Vector for Looping
  std::vector <double> norm_particle_weight;   // norm particle vec

  // Step 1: Normalize Weights
  // Repopulate the weights with normalized weights (reference 51:00 Min in Lecture 2021.09.22)
  for (int k {0}; k < input_vec_length; k++)
  {
    // particles_[k].weight = abs(exp(particles_[k].weight - max_particle_weight));
    norm_particle_weight.push_back(abs(exp(particles_[k].weight - max_particle_weight)));
    total_weight += norm_particle_weight[k];
  }

  // Step 2: Loop through each weight and resample
  // Main Loop; RESAMPLE
  for (int i {0}; i < num_of_resamples; i++)
  {   
      // reset comparison variable for bin navigation
      weight_sum = 0;

      // get a random number
      float random_num = rng_.UniformRandom(0, total_weight);
      //std::cout << "\n[Iter: " << i << "]\n Random Number: " << random_num << std::endl; // _____________ debug

      // loop through each particle weight/bucket
      for (int j {0}; j < input_vec_length; j++)
      {
          // zero case
          if (random_num == 0.00) 
          {
              reduced_particle_vec.push_back(particles_[0]);
              //std::cout << " Particle (" << iter << ") equaled min; added to output vec" << std::endl; // debug
              break;
          }
          // max case
          else if (random_num == total_weight)
          {
              reduced_particle_vec.push_back(particles_[input_vec_length-1]);
              //std::cout << " Particle (" << iter << ") equaled max; added to output vec" << std::endl; // debug
              break;
          }
          // middle bucket cases; keep adding buckets if the random number is not equal
          else if (random_num > weight_sum)
          {
              // Add a weight
              weight_sum += norm_particle_weight[j];

              // Check if random number is on the edge of two buckets
              if (random_num == weight_sum)
              {
                  k = (norm_particle_weight[j] <= norm_particle_weight[j+1]) ? j+1 : j; 
                  reduced_particle_vec.push_back(particles_[k]);
                  //std::cout << " Particle (" << iter << ") on edge; added to output vec" << std::endl; // debug
              }
              // Check if random number is less than the next bucket, and add to output vector
              else if (random_num < weight_sum)
              {
                  reduced_particle_vec.push_back(particles_[j]);
                  //std::cout << " Particle (" << iter << ") added to output vec" << std::endl; // ________ debug
              }
          }
      }
  }
  // Erase existing particles_ vector and fill with resampled particle vector (reduced_particle_vec)
  particles_.erase(particles_.begin(), particles_.end());
  particles_ = reduced_particle_vec;

}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.

  // Check to make sure particles are populated, if not do nothing
  if (particles_.empty() || odom_initialized_ == false)
  { 
    return;
  }
  
  
  // Call Update Every n'th Predict
  if (predict_steps == 5)
  {
    for(auto &particle : particles_)
    {
      // Call to Update
      Update(ranges, range_min, range_max, angle_min, angle_max, &particle);

      // Update Max Log Particle Weight Based On Return from Update
      if (particle.weight > max_particle_weight)
        max_particle_weight = particle.weight;
    }
    predict_steps = 0;  // Reset Number of Predicted Steps DOne
    updates_done++;     // Increment how many times Update has been called to determine when to call Resample
  }

  // Call Resample Every n'th Update
  if(updates_done == 10)
  {
    // Call to Resample
    Resample();

    // Reset Number of Updates Done
    updates_done = 0;
  }
  
}


void ParticleFilter::Predict(const Eigen::Vector3d &odom_cur) {
  
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
  
  // Reference: Table 5.6 in Sebastian Thrun, Wolfram Bugard & Dieter Fox
  // Calculate Difference Between New and Old Odom Readings

  // Variance Parameters, set_parameter
  double a1 = 8;  // 0.08 // angle 
  double a2 = 8;  //0.01; // angle 
  double a3 = 20;  //0.1; // trans
  double a4 = 12;  //0.1; // trans

  // Calculate Relative Odometry
  double del_x = odom_cur(0)-odom_old(0);
  double del_y = odom_cur(1)-odom_old(1);
  double del_rot = odom_cur(2)-odom_old(2);

  // Calculate Deltas based on Odom Data
  double del_rot_1 = atan2(del_y,del_x) - odom_old(2);
  double del_trans = sqrt(pow(del_y,2) + pow(del_x,2));
  double del_rot_2 = odom_cur(2) - odom_old(2) - del_rot_1;

  // Capture Length of Particle Vector
  int particle_vector_length = particles_.size();

  if (odom_initialized_ == true and del_trans != 0 and del_trans < 1 and del_rot < M_PI/2 and del_rot > -M_PI/2)
  {
    // std::cout << "\n\nPREDICT CALLED" << std::endl;
    for (int i {0}; i < particle_vector_length; i++) 
    {   
      // Calculate Variance
      double rot1_hat_var = rng_.Gaussian(0,(a1*pow(del_rot_1,2) + a2*pow(del_trans,2)));
      double trans_hat_var = rng_.Gaussian(0,(a3*pow(del_trans,2)+a4*pow(del_rot_1,2)+a4*pow(del_rot_2,2)));
      double rot2_hat_var = rng_.Gaussian(0,(a1*pow(del_rot_1,2) + a2*pow(del_trans,2)));

      // Relative Odometry with some Variance
      double del_rot_1_hat = get_angle_diff(del_rot_1,rot1_hat_var);
      double del_trans_hat = del_trans - trans_hat_var;
      double del_rot_2_hat = get_angle_diff(del_rot_2,rot2_hat_var);

      // Predicted State_t
      particles_[i].loc.x() = particles_[i].loc.x() + del_trans_hat*cos(particles_[i].angle+ del_rot_1_hat);
      particles_[i].loc.y() = particles_[i].loc.y() + del_trans_hat*sin(particles_[i].angle + del_rot_1_hat);
      particles_[i].angle = particles_[i].angle + del_rot_1_hat + del_rot_2_hat;

      // Set Current Odom to the Old Odom
      odom_old(0) = odom_cur(0); // x odom
      odom_old(1) = odom_cur(1); // y odom
      odom_old(2) = odom_cur(2); // angle odom
    }
    predict_steps++;             // keep track of how many predicts we did for call to update
  }
  else
  {
    // Set current odom values to previous values for next call to predict
    odom_old(0) = odom_cur(0);  // x odom
    odom_old(1) = odom_cur(1);  // y odom
    odom_old(2) = odom_cur(2);  // angle odom
  }
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {

  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.

  // Load Desired Map
  map_.Load(map_file);

  // Clear out particle vector to start fresh
  particles_.clear();

  // Initialize Variables
  Particle init_particle_cloud;
  odom_initialized_ = true;
  predict_steps = 1;
  //int num_of_init_particle_cloud = 50;  // set_parameter; number of particles to initialize with

  // Debug Matrix
  std::vector <Eigen::Vector2f> particle_matrix;
  for (float t {0}; t < 0.8; )
  {
    for (float s {0}; s < 0.8; )
    {
      particle_matrix.push_back(Eigen::Vector2f (s, t));
      s += 0.1;
    }
    t += 0.1;
  }

  // set_parameter for Gaussian standard deviation and mean
  for(auto particle: particle_matrix){
    init_particle_cloud.loc.x() = loc.x() + particle.x();
    init_particle_cloud.loc.y() = loc.y() + particle.y();
    init_particle_cloud.angle = angle;
    init_particle_cloud.weight = 0;
    particles_.push_back(init_particle_cloud);
  }

}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;

  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:


  if (odom_initialized_ == true)
  {
    // Initialize Variables
    double sum_x {0};
    double sum_y {0};
    double sum_cos_theta {0};
    double sum_sin_theta {0};
    double total_particle_weight {0};

    // Summations for all variables
    for (auto particles:particles_)
    {
      // Normalize Particle Weights
      double particle_norm_weight = exp(particles.weight - max_particle_weight);
      std::cout << "Particle_norm_weight: " << particle_norm_weight << std::endl;

      // Add All Normalized Particle Weights
      total_particle_weight += particle_norm_weight;

      // Calculate Numertors for Output location
      sum_x += particles.loc.x() * particle_norm_weight;
      sum_y += particles.loc.y() * particle_norm_weight;
      sum_cos_theta += cos(particles.angle) * particle_norm_weight;
      sum_sin_theta += sin(particles.angle) * particle_norm_weight;
      
    }

    // Averages for x, y, and theta;
    loc.x() = sum_x/total_particle_weight;
    loc.y() = sum_y/total_particle_weight;
    angle = atan2(sum_sin_theta/total_particle_weight,sum_cos_theta/total_particle_weight);
  }
}


}  // namespace particle_filter