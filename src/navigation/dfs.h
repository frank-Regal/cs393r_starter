#include<iostream>
#include<list>
#include<vector>
#include<stack>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

using namespace std;

class Graph {

private:

    int nodes;
    list<int>* adjlist;
    vector<bool> visited;

public:

    Graph() {
    }

    Graph(int nodes) { // Allocate resources 
        adjlist = new list<int>[nodes];
        visited.resize(nodes, false);
        this->nodes = nodes;
    }

    ~Graph() { // Free allocated resources
        delete[] adjlist;
    }

    void AddEdge(int src, int dst) {
        adjlist[src].push_back(dst);
        adjlist[dst].push_back(src);
    }

    // DFS recursive
    void DFS(int src) {
        visited[src] = true;
        cout << src << " ";
        for (auto& adj_node : adjlist[src]) {
            if (!visited[adj_node]) {
                DFS(adj_node);
            }
        }
    }

    // DFS iterative
    void DFS_Iterative(int src) {

        stack<int> stk;
        visited[src] = true;
        stk.push(src);

        while (!stk.empty()) {
            src = stk.top();
            stk.pop();
            cout << src << " ";
            for (auto& adj_node : adjlist[src]) {
                if (!visited[adj_node]) {
                    visited[adj_node] = true;
                    stk.push(adj_node);
                }
            }
        }
    }

    // Mark nodes unvisited for next traversal
    void MarkUnvisited() {
        fill(visited.begin(), visited.end(), false);
    }
};