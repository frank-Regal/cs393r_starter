
#include <vector>

#include "eigen3/Eigen/Dense"
#include "shared/util/random.h"

#ifndef RRT_GRAPH_H
#define RRT_GRAPH_H

class RRTGraph
{
private:
    std::vector<Eigen::Vector2f> graph;
    Eigen::Vector2f init_node;
    util_random::Random rng_;
    Eigen::Vector2f random;

public:

RRTGraph();

void SetInitNode(const Eigen::Vector2f& node);

Eigen::Vector2f GetInitNode();

void AddVertex(Eigen::Vector2f q_new);

void AddEdge(Eigen::Vector2f q_near, Eigen::Vector2f q_new);

Eigen::Vector2f GetRandq(const float Cx, const float Cy);

Eigen::Vector2f GetClosestq(const Eigen::Vector2f q_rand);

Eigen::Vector2f GetNewq(const Eigen::Vector2f q_near, const Eigen::Vector2f q_rand, const float delta_q);

void TestFunc();

};

#endif // RRT_GRAPH_H
