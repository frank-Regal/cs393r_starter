#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/util/random.h"
#include "rrt_graph.h"
#include <iostream>

using Eigen::Vector2f;
using std::string;
using std::vector;

RRTGraph::RRTGraph():
    init_node(0,0), random(0,0)
{

}

void RRTGraph::SetInitNode(const Eigen::Vector2f& node)
{
    init_node << node;
    graph.push_back(init_node);
    
}

Eigen::Vector2f RRTGraph::GetInitNode()
{
    return init_node;
}

void RRTGraph::AddVertex(Eigen::Vector2f q_new)
{
    std::cout << "TODO: Add Vertex" << std::endl;
}

void RRTGraph::AddEdge(Eigen::Vector2f q_near, Eigen::Vector2f q_new)
{
    std::cout << "TODO: Add Edge" <<std::endl;
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
    graph.push_back(Eigen::Vector2f(14,10));
    graph.push_back(Eigen::Vector2f(-12,4));
    graph.push_back(Eigen::Vector2f(-5,-11.21));
    graph.push_back(Eigen::Vector2f(2.23,16.7));

    for (auto& q:graph){
        delta_q = q_rand - q;
        magnitude = delta_q.norm();
        if (magnitude < min_dist){
            closest_q = q;
            min_dist = magnitude;
        }
    }

    return closest_q;
}

Eigen::Vector2f RRTGraph::GetNewq(const Eigen::Vector2f q_near, const Eigen::Vector2f q_rand, const float delta_q)
{
    return Eigen::Vector2f (0,0);
}

void RRTGraph::TestFunc()
{
    std::cout <<"testing 1, 2" << std::endl;
} 