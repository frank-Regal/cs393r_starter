#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/util/random.h"
#include "shared/math/geometry.h"
#include "rrt_graph.h"
#include <iostream>
#include <utility>
#include <deque>

using Eigen::Vector2f;
using std::string;
using std::vector;

RRTGraph::RRTGraph():
    init_node(0,0), random(0,0)
{

}

void RRTGraph::ClearTree()
{
    tree_vertex.clear();
    tree_edge.clear();
}

void RRTGraph::SetInitNode(const Eigen::Vector2f& node)
{
    init_node << node;
    tree_vertex.push_back(init_node);
}

Eigen::Vector2f RRTGraph::GetInitNode()
{
    return init_node;
}

void RRTGraph::AddVertex(Eigen::Vector2f q_new)
{
    tree_vertex.push_back(q_new);
}

void RRTGraph::AddEdge(Eigen::Vector2f q_near, Eigen::Vector2f q_new)
{
    std::vector<Eigen::Vector2f> tree_pair;
    tree_pair.push_back(q_near);
    tree_pair.push_back(q_new);
    tree_edge.push_back(tree_pair);
}

Eigen::Vector2f RRTGraph::GetRandq(float Cx, float Cy)
{
    random.x() = rng_.UniformRandom(-Cx, Cx);
    random.y() = rng_.UniformRandom(-Cy, Cy);
    return random;
}

Eigen::Vector2f RRTGraph::GetClosestq(const Eigen::Vector2f q_rand)
{
    Eigen::Vector2f delta_q (0,0);
    Eigen::Vector2f closest_q (0,0);
    float magnitude {0};
    float min_dist {500000};  //potential bug

    //TEST
    tree_vertex.push_back(Eigen::Vector2f(14,10));
    tree_vertex.push_back(Eigen::Vector2f(-12,4));
    tree_vertex.push_back(Eigen::Vector2f(-5,-11.21));
    tree_vertex.push_back(Eigen::Vector2f(2.23,16.7));

    for (auto& q:tree_vertex){
        delta_q = q_rand - q;
        magnitude = delta_q.norm();
        if (magnitude < min_dist){
            closest_q = q;
            min_dist = magnitude;
        }
    }

    return closest_q;
}

Eigen::Vector2f RRTGraph::GetNewq(const Eigen::Vector2f q_near, const Eigen::Vector2f q_rand, const float max_delta_q)
{
    Eigen::Vector2f q = q_rand;    // output vector
    Eigen::Vector2f delta_q (0,0); // delta between nodes
    float magnitude {0};           // magnitude of nodes
    float angle {0};               // angle of magnitudes

    delta_q = q_rand - q_near;
    magnitude = delta_q.norm();

    if (magnitude > max_delta_q){   
        angle = geometry::Angle(delta_q);
        q.x() = max_delta_q*cos(angle);
        q.y() = max_delta_q*sin(angle);
    }

    return q;
}

bool RRTGraph::IsNearGoal(const Eigen::Vector2f q, const Eigen::Vector2f q_goal, const float threshold)
{
    bool is_close = false;
    Eigen::Vector2f delta_q = q_goal - q;
    float magnitude = delta_q.norm();
    if (magnitude < threshold)
        is_close = true;

    return is_close; 
}

void RRTGraph::FindShortestPath(const Eigen::Vector2f q_near, const Eigen::Vector2f q_new)
{
    int tree_length = tree_edge.size();
    Eigen::Vector2f point_buff = q_near;

    for (int i {tree_length}; i > 0; i--){

        if (tree_edge[i][1] == point_buff){

            if (tree_edge[i][0] == init_node){
                path_to_goal.push_front(tree_edge[i][0]);
            }

        }
        
    }
}

void RRTGraph::TestFunc()
{
    std::cout <<"testing 1, 2" << std::endl;
} 