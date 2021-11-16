#include "dfs.h"
/*
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

int main()
{   

    Graph g(7);
    g.AddEdge(0, 1);
    g.AddEdge(0, 2);
    g.AddEdge(1, 3);
    g.AddEdge(1, 4);
    g.AddEdge(2, 3);
    g.AddEdge(3, 5);
    g.AddEdge(4, 6);
    g.AddEdge(5, 6);

    cout << "DFS Recursive Graph Traversal " << endl;
    cout << "Source Node (0): "; g.DFS(0);
    g.MarkUnvisited(); cout << endl;
    cout << "Source Node (3): "; g.DFS(3);
    g.MarkUnvisited(); cout << endl;

    cout << "DFS Iterative Graph Traversal" << endl;
    cout << "Source Node (0): "; g.DFS_Iterative(0);
    g.MarkUnvisited(); cout << endl;
    cout << "Source Node (3): "; g.DFS_Iterative(3);
    g.MarkUnvisited(); cout << endl << endl;
    
    Graph t(10);
    t.AddEdge(0, 1);
    t.AddEdge(0, 6);
    t.AddEdge(0, 7);
    t.AddEdge(1, 2);
    t.AddEdge(1, 4);
    t.AddEdge(1, 5);
    t.AddEdge(2, 3);
    t.AddEdge(7, 8);
    t.AddEdge(7, 9);

    cout << "DFS Recursive Tree Traversal" << endl;
    cout << "Root Node (0): "; t.DFS(0);
    t.MarkUnvisited(); cout << endl;
    cout << "Root Node (4): "; t.DFS(4);
    t.MarkUnvisited(); cout << endl;

    cout << "DFS Iterative Tree Traversal" << endl;
    cout << "Source Node (0): "; t.DFS_Iterative(0);
    t.MarkUnvisited(); cout << endl;
    cout << "Source Node (4): "; t.DFS_Iterative(4);
    t.MarkUnvisited(); cout << endl;
    return 0;

}
*/
