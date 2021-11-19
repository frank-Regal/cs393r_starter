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
\file    navigation_main.cc
\brief   Main entry point for reference Navigation implementation
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Localization2DMsg.h"
#include "gflags/gflags.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "visualization/visualization.h"

#include "navigation.h"

using amrl_msgs::Localization2DMsg;
using math_util::DegToRad;
using math_util::RadToDeg;
using navigation::Navigation;
using ros::Time;
using ros_helpers::Eigen3DToRosPoint;
using ros_helpers::Eigen2DToRosPoint;
using ros_helpers::RosPoint;
using ros_helpers::SetRosVector;
using std::string;
using std::vector;
using Eigen::Vector2f;
using visualization::DrawArc;
using visualization::DrawPoint;
using visualization::DrawLine;
using visualization::DrawParticle;

// Create command line arguments
DEFINE_string(laser_topic, "scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "odom", "Name of ROS topic for odometry data");
DEFINE_string(loc_topic, "localization", "Name of ROS topic for localization");
DEFINE_string(init_topic,
              "initialpose",
              "Name of ROS topic for initialization");
DEFINE_string(map, "maps/GDC1.txt", "Name of vector map file");
// RRT Add 
DEFINE_string(set_pose_topic,
              "/set_pose",
              "Name of ROS topic for initialization");


bool run_ = true;
bool set_pose_ = false; // RRT Add 
Eigen::Vector2f init_pose_ (0,0); // RRT Add
sensor_msgs::LaserScan last_laser_msg_;
Navigation* navigation_ = nullptr;

void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f, dt=%f\n",
           msg.header.stamp.toSec(),
           GetWallTime() - msg.header.stamp.toSec());
  }
  // Location of the laser on the robot. Assumes the laser is forward-facing.
  const Vector2f kLaserLoc(0.2, 0);

  static vector<Vector2f> point_cloud_; //RRT Add
  // Convert the LaserScan to a point cloud
  // The LaserScan parameters are accessible as follows:
  // msg.angle_increment // Angular increment between subsequent rays
  // msg.angle_max // Angle of the first ray
  // msg.angle_min // Angle of the last ray
  // msg.range_max // Maximum observable range
  // msg.range_min // Minimum observable range
  // msg.ranges[i] // The range of the i'th ray
  
  static vector<Vector2f> cloud(msg.ranges.size());
  
  for(std::size_t i = 0; i < msg.ranges.size(); i++){

    // Polar to Cartesian conversion, transforms to base link frame
    float angle = msg.angle_min + msg.angle_increment*i;
    float x = msg.ranges[i]*cos(angle) + kLaserLoc[0];
    float y = msg.ranges[i]*sin(angle) + kLaserLoc[1];
    
    cloud[i] = Vector2f(x, y);
  }

  navigation_->ObservePointCloud(cloud, msg.header.stamp.toNSec()); // Upstream uses .toSec()?
  last_laser_msg_ = msg;
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  //msg.header.stamp = ros::Time::now();
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  navigation_->UpdateOdometry(
      Vector2f(msg.pose.pose.position.x, msg.pose.pose.position.y),
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
      Vector2f(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
      msg.twist.twist.angular.z,
      ros::Time::now().toNSec());
}

void GoToCallback(const geometry_msgs::PoseStamped& msg) {
  const Vector2f loc(msg.pose.position.x, msg.pose.position.y);
  const float angle =
      2.0 * atan2(msg.pose.orientation.z, msg.pose.orientation.w);
  printf("Goal: (%f,%f) %f\u00b0\n", loc.x(), loc.y(), angle);
  navigation_->SetNavGoal(loc, angle);

  // RRT Add
  if (set_pose_ == true){
    std::cout << "Building RRT..." << std::endl;
    navigation_->BuildRRT(init_pose_, loc);
  }
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

void LocalizationCallback(const amrl_msgs::Localization2DMsg msg) {
  if (FLAGS_v > 0) {
    printf("Localization t=%f\n", GetWallTime());
  }
  navigation_->UpdateLocation(Vector2f(msg.pose.x, msg.pose.y), msg.pose.theta);
}

// RRT Add
void InitCallback(const amrl_msgs::Localization2DMsg& msg) {
  const Vector2f init_loc(msg.pose.x, msg.pose.y);
  const float init_angle = msg.pose.theta;
  const string map = "maps/" + msg.map + ".txt";
  printf("Initialized Location: (%f,%f) %f\u00b0\n",
         init_loc.x(),
         init_loc.y(),
         RadToDeg(init_angle));
  init_pose_ = init_loc;
  
  // Added for RRT
  navigation_->InitMap(map.c_str());

  set_pose_ = true;

}

void ProcessLive(ros::NodeHandle* n) {
  std::cout << "Here we go again..." << std::endl;
  ros::Subscriber initial_pose_sub = n->subscribe(
      FLAGS_set_pose_topic.c_str(),
      1,
      InitCallback);

  while (ros::ok() && run_) {
    ros::spinOnce();
    Sleep(0.01);
  }
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);

  // Initialize ROS.
  ros::init(argc, argv, "navigation", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  navigation_ = new Navigation(FLAGS_map, &n);

  ros::Subscriber velocity_sub =
      n.subscribe(FLAGS_odom_topic, 1, &OdometryCallback);
  ros::Subscriber localization_sub =
      n.subscribe(FLAGS_loc_topic, 1, &LocalizationCallback);
  ros::Subscriber laser_sub =
      n.subscribe(FLAGS_laser_topic, 1, &LaserCallback);
  ros::Subscriber goto_sub =
      n.subscribe("/move_base_simple/goal", 1, &GoToCallback);
  ros::Subscriber initial_pose_sub = 
      n.subscribe(FLAGS_set_pose_topic.c_str(), 1, &InitCallback);

  RateLoop loop(20.0);
  while (run_ && ros::ok()) {
    ros::spinOnce();
    navigation_->Run();
    loop.Sleep();
  }
  delete navigation_;
  return 0;
}
