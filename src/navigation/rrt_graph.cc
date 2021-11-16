#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/util/random.h"
#include "shared/math/geometry.h"
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
    tree.clear();     // clear out previous tree 
    init_node << node;
    tree.push_back(init_node);
}

Eigen::Vector2f RRTGraph::GetInitNode()
{
    return init_node;
}

void RRTGraph::AddVertex(Eigen::Vector2f q_new)
{
    tree.push_back(q_new);
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
    tree.push_back(Eigen::Vector2f(14,10));
    tree.push_back(Eigen::Vector2f(-12,4));
    tree.push_back(Eigen::Vector2f(-5,-11.21));
    tree.push_back(Eigen::Vector2f(2.23,16.7));

    for (auto& q:tree){
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

void RRTGraph::TestFunc()
{
    std::cout <<"testing 1, 2" << std::endl;
} 