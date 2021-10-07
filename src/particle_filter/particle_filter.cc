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
  int updates_done {0};
}

namespace {
  int predict_steps {0};
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
    odom_initialized_(false) {}

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
  int step_size_of_scan {10};

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

  // // Adding tuning params for Min and Max Ranges of Physical Laser Scanner
  // float min_range_with_tuning = range_min + range_min * min_dist_tuning;
  // float max_range_with_tuning = range_max - range_max * max_dist_tuning;

  // Standard deviation of Physical LIDAR System; set_parameter
  double ray_std_dev = 0.1;

  // Reset variables between each point cloud
  float weight {0};
  float max_weight {0};
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
      weight = 0;
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

    // Calculate Max Weight for log calculation
    if(weight > max_weight)
      max_weight = weight;
  }
  particle.weight = total_weight;
  //std::cout << "Particle Weight Max: " << max_weight << std::endl; // debug
  //std::cout << "Particle Weight Total: " << particle.weight << std::endl; //debug
  //particle.weight = particle.weight/max_weight;
  //std::cout << "Normalized Particle Weights: " << particle.weight << std::endl; // debug
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // Predefine the Number of Resamples; set_parameter
  int num_of_resamples {30};

  // Initializations
  std::vector <Particle> reduced_particle_vec; // return vector (vector of the kept particles)
  double weight_sum {0};                       // comparison variable 
  double total_weight {0};                     // total weight of all particles
  double max_weight {0};
  int k {0};                                   // edge case variable

  
  // Get Length of Input Vector for Looping
  int input_vec_length = particles_.size();

  // Step 1: Normalize Weights
  //  a. Compute Sum of All Weights
  for (auto get_particle: particles_)
  {
      if (get_particle.weight > max_weight)
        max_weight = get_particle.weight;
      
  }
  //std::cout << "max_weight: " << max_weight << std::endl;

  //  b. Repopulate the weights with normalized weights (reference 51:00 Min in Lecture 2021.09.22)
  for (int k {0}; k < input_vec_length; k++)
  {
    //std::cout << "Unormalized Particles: " << particles_[k].weight << std::endl;
    particles_[k].weight = abs(particles_[k].weight - max_weight);
    //std::cout << "Normalized Particles: " << particles_[k].weight << std::endl;
  }

  // Step 2: Compute Sum of Normalized Weights
  // Compute Sum of Particle Weights to Get Upper Container Bound
  for (auto get_particle: particles_)
  {
      total_weight += get_particle.weight;
      //std::cout << "total_weight: " << total_weight << std::endl;
  }

  // Step 3: Loop through each weight and resample
  // Main Loop; RESAMPLE
  for (int i {0}; i < num_of_resamples; i++)
  {   
      // reset comparison variable
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
              weight_sum += particles_[j].weight;

              // Check if random number is on the edge of two buckets
              if (random_num == weight_sum)
              {
                  k = (particles_[j].weight <= particles_[j+1].weight) ? j+1 : j; 
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

      // Erase existing particles_ vector and fill with resampled particle vector (reduced_particle_vec)
      // CHECK 
      //particles_.erase(particles_.begin(), particles_.end());
      //particles_ = reduced_particle_vec;
  }
  particles_.erase(particles_.begin(), particles_.end());
  particles_ = reduced_particle_vec;
  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  // float x = rng_.UniformRandom(0, 1);
  // printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
  //        x);
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.

  if (predict_steps == 0)
  {
    std::cout << "Called Update" << std::endl;
    std::cout << "updates_done: " << updates_done << std::endl;
    for(auto &particle : particles_)
    {
      Update(ranges, range_min, range_max, angle_min, angle_max, &particle);
      // std::cout << "\n\n[PARTICLE VALUES ]"  //debug
      //             << "\n x: " << particle.loc.x()
      //             << "\n y: " << particle.loc.y()
      //             << "\n angle: " << particle.angle
      //             << "\n weight: " << particle.weight
      //             << std::endl;
    }
    predict_steps = 1;
    updates_done++;
  }
  //TODO: set_parameter for number of updates done between each resampling
  if(updates_done == 3)
  {
    std::cout << "***called resample" << std::endl;
    Resample();
    updates_done = 0;
    predict_steps = 0;
    // std::cout << "\n\n[PARTICLE VALUES ]"  //debug
    //           << "\n x: " << particle.loc.x()
    //           << "\n y: " << particle.loc.y()
    //           << "\n angle: " << particle.angle
    //           << "\n weight: " << particle.weight
    //           << std::endl;
  }
}

void ParticleFilter::Predict(const Eigen::Vector3d &odom_cur) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
  
  // Reference: Table 5.6 in Sebastian Thrun, Wolfram Bugard & Dieter Fox
  // Calculate Difference Between New and Old Odom Readings

  // std::cout << "ODOM READINGS"
  //           << "\n cur x: " << odom_cur(0)
  //           << "\n cur y: " << odom_cur(1)
  //           << "\n cur t: " << odom_cur(2)
  //           << "\n old x: " << odom_old(0)
  //           << "\n old y: " << odom_old(1)
  //           << "\n old t: " << odom_old(2) << std::endl;

    double a1 = 0.08;
    double a2 = 0.01;
    double a3 = 0.1;
    double a4 = 0.1;

    double del_x = odom_cur(0)-odom_old(0);
    double del_y = odom_cur(1)-odom_old(1);

    // pow pow
    double del_rot_1 = get_angle_diff(atan2((odom_cur(1)-odom_old(1)),(odom_cur(0)-odom_old(0))),odom_old(2));
    double del_trans = sqrt(pow(del_y,2) + pow(del_x,2));
    double del_rot_2 = get_angle_diff(odom_cur(2),odom_old(2)) - del_rot_1;
    int particle_vector_length = particles_.size();
  std::cout << "predict_steps: " << predict_steps << std::endl;
  if (odom_initialized_ and (del_trans < 1)  and odom_cur(2) < M_PI/2 and odom_cur(2) > -M_PI/2 and predict_steps == 5)
  {
    std::cout << "\n\nPREDICT CALLED" << std::endl;
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

    odom_old(0) = odom_cur(0); // x odom
    odom_old(1) = odom_cur(1); // y odom
    odom_old(2) = odom_cur(2); // angle odom
    predict_steps = 0;
    }
  }
    
    // ****************************************************************************************************
    /*
    //std::cout << "odom initialized" << std::endl;
    // Loop through current list of particles and update the particle location vector based on odom readings
    for (int i {0}; i < length_of_particles_vec; i++)
    {
      // Convert to Map Coordinates
      double rotation_angle = get_angle_diff(particles_[i].angle, odom_old(2));

      // Rotate Previous Odom about Last Known Particle Location
      double map_frame_del_x = del_x*cos(rotation_angle) - del_y*sin(rotation_angle);
      double map_frame_del_y = del_x*cos(rotation_angle) + del_y*sin(rotation_angle);
      double map_frame_del_trans = sqrt(pow(map_frame_del_x,2) + pow(map_frame_del_y,2));

      // Add Variance to deltas
      double del_x_hat = map_frame_del_x + rng_.Gaussian(0, ( a1*map_frame_del_trans + a2*abs(del_theta) )); 
      double del_y_hat = map_frame_del_y + rng_.Gaussian(0, ( a1*map_frame_del_trans + a2*abs(del_theta) )); 
      double del_theta_hat = del_theta + rng_.Gaussian(0, ( a3*map_frame_del_trans + a4*abs(del_theta) )); 

      //std::cout << "PARTICLE VALUES (0) = "   // debug
      //          << "x0: " << particles_[i].loc.x()
      //          << " y0: " << particles_[i].loc.y()
      //          << " angle0: " << particles_[i].angle
      //          << std::endl;
      
      // Update Particle Location
      particles_[i].loc.x() += del_x_hat;
      particles_[i].loc.y() += del_y_hat;
      particles_[i].angle += del_theta_hat;

      //std::cout << "PARTICLE VALUES (1) = "  //debug
      //          << "x1: " << particles_[i].loc.x()
      //          << " y1: " << particles_[i].loc.y()
      //          << " angle1: " << particles_[i].angle
      //          << std::endl;

    }
    */
    // *********************************************************************************************
  else
  {
    // Set current odom values to previous values for next call to predict
    odom_old(0) = odom_cur(0); // x odom
    odom_old(1) = odom_cur(1); // y odom
    odom_old(2) = odom_cur(2); // angle odom
    odom_initialized_ = true;
    predict_steps++;
  }
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);

  Particle init;

  // TODO: set_parameter for Gaussian standard deviation and mean
  for(int i {0}; i < 50; i++){
    init.loc.x() = loc.x() + rng_.Gaussian(0.0, 0.1);
    init.loc.y() = loc.y() + rng_.Gaussian(0.0, 0.1);
    init.angle = angle + rng_.Gaussian(0.0, 0.1);
    init.weight = 0;
    particles_.push_back(init);
  }

}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  //loc = Vector2f(0, 0);
  //angle = 0;

  // Get Total Length of Input Vector
    int vector_length = particles_.size();
    
    //if (vector_length == 0)
    //  vector_length =1;

    //std::cout << "Get Location Called" << std::endl;

    // Initializations
    double sum_x {0};
    double sum_y {0};
    double sum_cos_theta {0};
    double sum_sin_theta {0};

    // Summations for all variables
    for (auto vec:particles_)
    {
        sum_x += vec.loc.x();
        sum_y += vec.loc.y();
        sum_cos_theta += cos(vec.angle);
        sum_sin_theta += sin(vec.angle);
    }

    // Averages for x, y, and theta;
    loc.x() = sum_x/vector_length;
    loc.y() = sum_y/vector_length;
    angle = atan2(sum_sin_theta/vector_length,sum_cos_theta/vector_length);
}


}  // namespace particle_filter