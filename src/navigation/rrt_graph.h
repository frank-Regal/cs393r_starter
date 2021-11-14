
#include <vector>

#include "eigen3/Eigen/Dense"

#ifndef RRT_GRAPH_H
#define RRT_GRAPH_H

class RRTGraph
{
private:
    std::vector<Eigen::Vector2f> graph;
    Eigen::Vector2f init_node;

public:

RRTGraph();

void SetInitNode(const Eigen::Vector2f& node);

Eigen::Vector2f GetInitNode();

void AddVertex(Eigen::Vector2f q_new);

void AddEdge(Eigen::Vector2f q_near, Eigen::Vector2f q_new);

Eigen::Vector2f GetRandq(float Cx, float Cy);

void TestFunc();

};

#endif // RRT_GRAPH_H
