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

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
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
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
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
    printf("P0: %f, %f P1: %f,%f\n", 
           my_line.p0.x(),
           my_line.p0.y(),
           my_line.p1.x(),
           my_line.p1.y());

    // Check for intersections:
    bool intersects = map_line.Intersects(my_line);
    // You can also simultaneously check for intersection, and return the point
    // of intersection:
    Vector2f intersection_point; // Return variable
    intersects = map_line.Intersection(my_line, &intersection_point);
    if (intersects) {
      printf("Intersects at %f,%f\n", 
             intersection_point.x(),
             intersection_point.y());
    } else {
      printf("No intersection\n");
    }
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
  double random_num {0};                       // normal random number variable
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
}

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.


  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  float x = rng_.Gaussian(0.0, 2.0);
  printf("Random number drawn from Gaussian distribution with 0 mean and "
         "standard deviation of 2 : %f\n", x);
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);
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

    // Averages for x, y, and theta; weight hard coded to 1
    loc.x() = sum_x/vector_length;
    loc.y() = sum_y/vector_length;
    angle = atan2(sum_sin_theta/vector_length,sum_cos_theta/vector_length);
}


}  // namespace particle_filter
