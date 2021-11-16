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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/math/geometry.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include "rrt_graph.h"
#include "shared/math/line2d.h"
#include "vector_map/vector_map.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using geometry::line2f;
using vector_map::VectorMap;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace



namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    tree_()
 {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"

  // Eventually, you will have to set the control values to issue drive commands:
  // drive_msg_.curvature = ...;
  drive_msg_.velocity = 0.9;

  Eigen::Vector2f random_num = tree_.GetRandq(20,20); //testing 1 2
  std::cout << "random_num: " << random_num << std::endl;
  Eigen::Vector2f closest_point = tree_.GetClosestq(random_num);
  std::cout << "closest_point: " << closest_point << "\n" << std::endl;
  float ang = geometry::Angle(closest_point);
  std::cout << "angle: " << ang << std::endl;

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

// Added the Following for RRT

void Navigation::InitMap(const string& map_file)
{ // Load Map File. Called in navigation_main.cc
  map_.Load(map_file);
  std::cout << "Initialized: " << map_file << std::endl;
}

Eigen::Vector2f Navigation::FindIntersection(const Eigen::Vector2f q_near, const Eigen::Vector2f q_new_cur)
{ // Set a new node if the map intersects

  line2f q_line (q_near.x(),q_near.y(), q_new_cur.x(), q_new_cur.y());
  Eigen::Vector2f output_q = q_new_cur;

  Eigen::Vector2f delta_q (0,0);
  Eigen::Vector2f closest_q (0,0);
  float magnitude {0};
  float min_dist {500000};  //potential bug


  for (size_t i {0}; i < map_.lines.size(); ++i){

    const line2f map_line = map_.lines[i];
    bool intersects = map_line.Intersection(q_line, &closest_q);
    
    if (intersects == true){
      delta_q = q_near - closest_q;
      magnitude = delta_q.norm();

      // find the shortest intersecting line
      if (magnitude < min_dist){
        output_q = closest_q;
        min_dist = magnitude;
      }
    }
  }

  return output_q;
}

void Navigation::BuildRRT(const Eigen::Vector2f q_init, const int k, const float delta_q)
{ // Rapidly Exploring Random Tree
  
  // Input Definitions:
  // - q_init = intial configuration
  // - k = number of vertices
  // - delta_q = incremental distance

  Eigen::Vector2f q_rand (0,0);  // random node within the c-space
  Eigen::Vector2f q_near (0,0);  // nears node
  Eigen::Vector2f q_trim (0,0);  // holder for new node
  Eigen::Vector2f q_new  (0,0);  // holder for new node
  Eigen::Vector2f C (20.0,20.0); // c-space / exploration space
  Eigen::Vector2f q_goal (0,0);  // TODO change to Set_Nav_Goal Variables 
  float goal_threshold = 0.5;    // meters
  bool goal_reached = false;     // check if robot is at goal yet

  tree_.SetInitNode(q_init); // initialize the starting point of rrt

  while(goal_reached == false)
  {
    q_rand = tree_.GetRandq(C.x(), C.y());            // get a random node
    q_near = tree_.GetClosestq(q_rand);               // get the closest node to the random node
    q_trim = tree_.GetNewq(q_near, q_rand, delta_q);  // get a new q based on the max distance TODO: Need to add wall checking
    q_new  = FindIntersection(q_near, q_trim);        // determine if the map intersects with the new node

    goal_reached = tree_.IsNearGoal(q_new, q_goal,goal_threshold);  // is the node within the threshold of the goal

    if (goal_reached == true){
      std::cout << "path found" << std::endl;
      tree_.FindShortestPath(q_near, q_new);
    } else {
      tree_.AddVertex(q_new);
      tree_.AddEdge(q_near,q_new);
    }

    
  }

}
}  // namespace navigation
