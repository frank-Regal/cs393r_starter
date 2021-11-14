#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "rrt_graph.h"
#include <iostream>

using Eigen::Vector2f;
using std::string;
using std::vector;

RRTGraph::RRTGraph() : init_node(0,0)
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
    std::cout << "Get Random Number" << std::endl;
    return Eigen::Vector2f (0,0);
}

void RRTGraph::TestFunc()
{
    std::cout <<"testing 1, 2" << std::endl;
} 