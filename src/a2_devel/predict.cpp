#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <random>
#include <chrono>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

// Helper Function: Random Number Generator
// CHECK
double get_random_double(int min, int max)
{
    // Set Bounds
    double lower_bound = min;
    double upper_bound = max;

    // Build Distribution of Range
    std::uniform_real_distribution<double> distribution(lower_bound,upper_bound);

    // Pseudo-random number engine initialization
    std::default_random_engine re;

    // ReSeed
    re.seed(std::chrono::system_clock::now().time_since_epoch().count());

    // Sample
    double a_random_double = distribution(re);

    return a_random_double;
}

// =============================================================================================================================
// MOTION MODEL
// 
// returns vector of where the current pose should be based on current & past odom and the last known pose

// Helper Function: calculates relative motion over new and old x, y, and theta readings from robot odom
// CHECK
Eigen::Vector3d get_relative_motion(Eigen::Vector3d odom_old, Eigen::Vector3d odom_cur)
{
    // Note: Input Vector3d Should Contain Data in the Following Order: [x, y, theta]
    // Reference: Table 5.6 in Sebastian Thrun, Wolfram Bugard & Dieter Fox
    double del_x {odom_cur(0)-odom_old(0)};
    double del_y {odom_cur(1)-odom_old(1)};
    double del_theta {odom_cur(2)-odom_old(2)};

    // Delta Rotation 1 
    double del_rot1 = atan2(del_y,del_x) - odom_old(2);

    // Delta Translation Between Current and Last
    double del_trans = sqrt(pow(del_x,2) + pow(del_y,2));

    // Delta Rotation 2
    double del_rot2 = del_theta - del_rot1;

    // Output
    Eigen::Vector3d relative_motion (del_rot1, del_trans, del_rot2);

    return relative_motion;
}

// Helper Function: sample a normal distribution
// CHECK
double sample_normal_dist(double variance)
{
    // Zero Mean SAMPLE Guassian Distribution With Input As The Variance
    // Reference: Table 5.4 in Sebastian Thrun, Wolfram Bugard & Dieter Fox

    // TODO; set_parameter
    int const sample_size {12}; 
    
    // Initialize
    double x {0};
    double std_dev {sqrt(variance)};
    
    // Definition of SAMPLE Gaussian Distribution
    for (int i {0}; i < sample_size; i++)
    {
        x = x + get_random_double(-std_dev,std_dev);
    }
    x = x * 0.5;

    return x;
}

// Main Function: Motion Model Definition
Eigen::Vector3d sample_motion_model_odometry(Eigen::Vector3d odom_old, Eigen::Vector3d odom_cur, Eigen::Vector3d pose_last)
{
    // Reference: Table 5.6 in Sebastian Thrun, Wolfram Bugard & Dieter Fox

    // Get Relative Motion Vector From Odom Readings 
    // fill rel_mot Vector3d: it will be in the following format [delta rot1 (0), delta trans (1), delta rot2 (2)]
    Eigen::Vector3d rel_mot (get_relative_motion(odom_old, odom_cur));
    
    // set alpha variance paramaters; set_parameter
    double a1 {0.4};
    double a2 {0.4};
    double a3 {0.4};
    double a4 {0.4};

    // get relative motion with variance factored in
    double del_rot_1_hat = rel_mot(0) - sample_normal_dist( a1*pow(rel_mot(0),2) + a2*pow(rel_mot(1),2) );
    double del_trans_hat = rel_mot(1) - sample_normal_dist( a3*pow(rel_mot(1),2) + a4*pow(rel_mot(0),2) + a4*pow(rel_mot(2),2) );
    double del_rot_2_hat = rel_mot(2) - sample_normal_dist( a1*pow(rel_mot(2),2) + a2*pow(rel_mot(1),2) );

    // initialize predicted pose variables; output variables
    double x_predict {0};
    double y_predict {0};
    double theta_predict {0};

    x_predict = pose_last.x() + del_trans_hat * cos( pose_last(2) + del_rot_1_hat );
    y_predict = pose_last.y() + del_trans_hat * sin( pose_last(2) + del_rot_1_hat );
    theta_predict = pose_last(2) + del_rot_1_hat + del_rot_2_hat;

    // build return vector
    Eigen::Vector3d predicted_state (x_predict, y_predict, theta_predict);

    return predicted_state;
}


int main (void) 
{
    // insanity check
    std::cout << "start" << std::endl;

    // spoofed odom data
    Eigen::Vector3d new_odom (8.0,7.0,3.1459/2);
    Eigen::Vector3d old_odom; //(6.0,7.0, 0);
    old_odom(0) = 6.0;
    old_odom(1) = 7.0;
    old_odom(2) = 0;
    
    // spoofed location data
    Eigen::Vector3d pose_last (15, 16, 0);

    // point predicted
    Eigen::Vector3d predicted_point;
    predicted_point << sample_motion_model_odometry(old_odom, new_odom, pose_last);
    std::cout << predicted_point << std::endl;

    return 0;
}