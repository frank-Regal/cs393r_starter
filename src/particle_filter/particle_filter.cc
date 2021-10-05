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
  int updates_done = 0;
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
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  vector<Vector2f> loc_of_max_distance_ray;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  
  /*
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) {
    scan[i] = Vector2f(0, 0);
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  for (size_t i = 0; i < map_.lines.size(); ++i) {
    const line2f map_line = map_.lines[i];
    // The line2f class has helper functions that will be useful.
    // You can create a new line segment instance as follows, for :
    line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
    // Access the end points using `.p0` and `.p1` members:
    //printf("P0: %f, %f P1: %f,%f\n", 
    //       my_line.p0.x(),
    //       my_line.p0.y(),
    //       my_line.p1.x(),
    //       my_line.p1.y());

    // Check for intersections:
    bool intersects = map_line.Intersects(my_line);
    // You can also simultaneously check for intersection, and return the point
    // of intersection:
    Vector2f intersection_point; // Return variable
    intersects = map_line.Intersection(my_line, &intersection_point);
    if (intersects) {
      // printf("Intersects at %f,%f\n", 
            //  intersection_point.x(),
            //  intersection_point.y());
    } else {
      //printf("No intersection\n");
    }
    */
    // ********
    
    // set_parameter
    int step_size_of_scan {10};
    // Reduce the input vectors size to account for every 10th laser scan ray
    int length_of_scan_vec = num_ranges/step_size_of_scan;
    scan.resize(length_of_scan_vec);
  
    // build vector of laser scan angles
    std::vector <double> predicted_scan_angles;

    // Calculations: Predicted Beginning and End Points of Each Ray within Theoretical Laser Scan
    // Points for Laser Scanner in Space using Distance between baselink and laser scanner on physical robot to be 0.2 meters
    double laser_scanner_loc_x = loc.x() + 0.2*cos(angle);
    double laser_scanner_loc_y = loc.y() + 0.2*sin(angle);
    Eigen::Vector2d laser_scanner_loc(laser_scanner_loc_x,laser_scanner_loc_y);


    // Starting Point
    double parsed_angle = angle+angle_min;

    // fill vector of scan angles
    for (int i {0}; i < length_of_scan_vec; i++)
    {
      parsed_angle += ((angle_max-angle_min)/length_of_scan_vec);
      predicted_scan_angles.push_back(parsed_angle);
    }

    // loop through each theoretical scan
    for (int j {0}; j < length_of_scan_vec; j++)
    {
      // Initialize the points of the predicted laser scan rays
      geometry::line2d laser_ray(1,2,3,4);
      laser_ray.p0.x() = laser_scanner_loc_x + range_min*cos(predicted_scan_angles[j]);
      laser_ray.p0.y() = laser_scanner_loc_y + range_min*sin(predicted_scan_angles[j]);
      laser_ray.p1.x() = laser_scanner_loc_x + range_max*cos(predicted_scan_angles[j]);
      laser_ray.p1.y() = laser_scanner_loc_y + range_max*sin(predicted_scan_angles[j]);
      
      // Fill i-th entry to return vector with the point of the max range
      scan[j] << laser_ray.p1.x(),
                 laser_ray.p1.y();
      
      double max_distance_of_current_ray = (laser_scanner_loc - scan[j]).norm();


      // Loop Through Each Map Lines from Imported Text File to See if Laser Rays Intersect
      for (size_t k = 0; k < map_.lines.size(); ++k) 
      {
        // Assign Map Lines to Variable
        const line2f map_line = map_.lines[k];
        
        // Initialize Return Variable
        Eigen::Vector2f intersection_point;

        // Compare Lines in Map to Theoretical Laser Rays from Each Particle
        bool intersects = map_line.Intersection(laser_ray, &intersection_point);
        if (interests)
        {
          // Grab Distance of that intersecting ray
          double distance_of_intersecting_ray = (intersection_point-laser_scanner_loc).norm;

          // If that distance is smaller than the current max ray distance for this ith scan, set it to the max_distance
          if ( distance_of_intersecting_ray < max_distance_of_current_ray)
          {
            max_distance_of_current_ray = distance_of_intersecting_ray; // max distance of current ray not interesting
            loc_of_max_distance_ray = intersection_point;
          }
        }
      }
      // fill with location of the max distance of a non-intersecting ray
      scan[j] = loc_of_max_distance_ray; 
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

  Particle& particle = *p_ptr;

  // Initialize point cloud vector
  vector<Vector2f> predicted_point_cloud;
  // Fill point cloud vector
  GetPredictedPointCloud(particle.loc, particle.angle, 
                         ranges.size(), range_min, range_max, 
                         angle_min, angle_max, &predicted_point_cloud);

  // Resize lidar scan range
  vector<float> resized_ranges(predicted_point_cloud.size());
  int lidar_ray_step_size = ranges.size() / predicted_point_cloud.size();

  int predicted_point_cloud_length = predicted_point_cloud.size();

  for(int i = 0; i < predicted_point_cloud_length; i++)
    resized_ranges[i] = ranges[lidar_ray_step_size*i];

  // Tuning parameters for the minimum and maximum distances of the laser scanner
  double min_dist_tuning = 0.5;
  double max_dist_tuning = 0.5;
  // Standard deviation of LIDAR
  double ray_std_dev = 0.125;

  // Reset variables between each point cloud
  float probability = 1;
  float weight_max = 0;

  for(int i = 0; i < predicted_point_cloud_length; i++)
  {
    // Laser scanner location is offset from the particle location
    float laser_scanner_loc_x = particle.loc.x() + 0.2*cos(particle.angle);
    float laser_scanner_loc_y = particle.loc.y() + 0.2*sin(particle.angle);

    // Distance laser scanner found to be from obstacle (actual distance)
    float particle_actual_distance = resized_ranges[i];

    // Calculate distance between the laser scanner and the returned point cloud location (theoretical distance)
    float theoretical_dist_x = predicted_point_cloud[i].x() - laser_scanner_loc_x;
    float theoretical_dist_y = predicted_point_cloud[i].y() - laser_scanner_loc_y;
    float particle_theoretical_distance = sqrt(pow(theoretical_dist_x, 2)+pow(theoretical_dist_y, 2));

    float min_range_with_tuning = range_min * min_dist_tuning;
    float max_range_with_tuning = range_max * max_dist_tuning;
    float delta_distance = particle_actual_distance - particle_theoretical_distance;

    // Function for weighting the probability of particle being in the location found
    if(particle_actual_distance < range_min or particle_actual_distance > range_max)
      probability = 0;
    else if (particle_actual_distance < particle_theoretical_distance - (min_range_with_tuning))
      probability = exp(-pow(min_range_with_tuning,2) / pow(ray_std_dev,2));
    else if (particle_actual_distance > particle_theoretical_distance + (max_range_with_tuning))
      probability = exp(pow(max_range_with_tuning,2) / pow(ray_std_dev,2));
    else
      probability = exp(pow(delta_distance,2) / pow(ray_std_dev,2));

    particle.weight = particle.weight + probability;

    // Calculate Max Weight for log calculation
    if(particle.weight > weight_max)
      weight_max = particle.weight;
  }

  particle.weight = log(particle.weight) - log(weight_max);
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
  int num_of_resamples {100};

  // Initializations
  std::vector <Particle> reduced_particle_vec; // return vector (vector of the kept particles)
  double weight_sum {0};                       // comparison variable 
  double total_weight {0};                     // total weight of all particles
  //double random_num {0};                       // normal random number variable
  int k {0};                                   // edge case variable

  // Get Length of Input Vector for Looping
  int input_vec_length = particles_.size();

  // Compute Sum of Particle Weights to Get Upper Container Bound
  for (auto get_particle: particles_)
      total_weight += get_particle.weight;
  
  // Main Loop; RESAMPLE
  for (int i {0}; i < num_of_resamples; i++)
  {   
      // reset comparison variable
      weight_sum = 0;

      // get a random number
      float random_num = rng_.UniformRandom(0, total_weight);
      std::cout << "\n[Iter: " << i << "]\n Random Number: " << random_num << std::endl; // _____________ debug

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
      particles_.erase(particles_.begin(), particles_.end());
      particles_ = reduced_particle_vec;
  }

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
  for(auto &particle : particles_)
  {
    Update(ranges, range_min, range_max, angle_min, angle_max, &particle);

  }

  //TODO: set_parameter for number of updates done between each resampling
  if(updates_done > 10)
  {
    Resample();
    updates_done = 0;
  }
  else 
    updates_done++;
}

void ParticleFilter::Predict(const Eigen::Vector3d &odom_cur) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.
  
  // Reference: Table 5.6 in Sebastian Thrun, Wolfram Bugard & Dieter Fox
  // Calculate Difference Between New and Old Odom Readings

  std::cout << "ODOM READINGS"
            << "\n cur x: " << odom_cur(0)
            << "\n cur y: " << odom_cur(1)
            << "\n cur t: " << odom_cur(2)
            << "\n old x: " << odom_old(0)
            << "\n old y: " << odom_old(1)
            << "\n old t: " << odom_old(2) << std::endl;

  
  double del_x = odom_cur(0)-odom_old(0);
  double del_y = odom_cur(1)-odom_old(1);
  double del_theta = get_angle_diff(odom_cur(2),odom_old(2));
  double del_trans = sqrt(pow(del_x,2) + pow(del_y,2));


  /*
  // Attempt 1 *********************************************************************************************
  double del_trans = sqrt(pow(del_x,2) + pow(del_y,2)); 
  // Delta Rotation 1 
  double del_rot1 = atan2(del_y,del_x) - odom_old(2);
  // Delta Translation Between Current and Last Odom Position
  double del_trans = sqrt(pow(del_x,2) + pow(del_y,2));
  // Delta Rotation 2
  double del_rot2 = del_theta - del_rot1;
  
  // set alpha variance paramaters; set_parameter
  double a1 {0};
  double a2 {0};
  double a3 {0};
  double a4 {0};

  // get relative motion with variance factored in
  // CHECK: Should this be minus?
  double del_rot_1_hat = del_rot1 + rng_.Gaussian(odom_cur(0), sqrt(a1*pow(del_rot1,2) + a2*pow(del_trans,2)) );
  double del_trans_hat = del_trans + rng_.Gaussian(odom_cur(1), sqrt(a3*pow(del_trans,2) + a4*pow(del_rot1,2) + a4*pow(del_rot2,2)) );
  double del_rot_2_hat = del_rot2 + rng_.Gaussian(odom_cur(2), sqrt(a1*pow(del_rot2,2) + a2*pow(del_trans,2)) );
  */
 
  // Attempt 2 *********************************************************************************************
  
  // set alpha variance paramaters; set_parameter
  double a1 {0};
  double a2 {0};
  double a3 {0};
  double a4 {0};

  // Get Total Length of particle vector
  int length_of_particles_vec = particles_.size();
  if (odom_initialized_ and del_trans < 1)
  {
    std::cout << "odom initialized" << std::endl;
    // Loop through current list of particles and update the particle location vector based on odom readings
    for (int i {0}; i < length_of_particles_vec; i++)
    {
      /* 
      // Attempt 1
      particles_[i].loc.x() = particles_[i].loc.x() + del_trans_hat * cos( particles_[i].angle + del_rot_1_hat );
      particles_[i].loc.y() = particles_[i].loc.y() + del_trans_hat * sin( particles_[i].angle + del_rot_1_hat );
      particles_[i].angle = particles_[i].angle + del_rot_1_hat + del_rot_2_hat;
      */

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

      std::cout << "PARTICLE VALUES (0) = "
                << "x0: " << particles_[i].loc.x()
                << " y0: " << particles_[i].loc.y()
                << " angle0: " << particles_[i].angle
                << std::endl;
      
      // Update Particle Location
      particles_[i].loc.x() += del_x_hat;
      particles_[i].loc.y() += del_y_hat;
      particles_[i].angle += del_theta_hat;

      std::cout << "PARTICLE VALUES (1) = "
                << "x1: " << particles_[i].loc.x()
                << " y1: " << particles_[i].loc.y()
                << " angle1: " << particles_[i].angle
                << std::endl;
    }

  }
  else
  {// Set current odom values to previous values for next call to predict
  odom_old(0) = odom_cur(0); // x odom
  odom_old(1) = odom_cur(1); // y odom
  odom_old(2) = odom_cur(2); // angle odom
  odom_initialized_ = true;
  updates_done = 0;
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
    
    if (vector_length == 0)
      vector_length =1;

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