//
// Created by tomst on 2-6-2024.
//

#ifndef CODE_MYGRAPH_H
#define CODE_MYGRAPH_H

#include <vector>
#include <iostream>
#include <unordered_map>
#include <memory>


struct Edge{
    int nodeId1;
    int nodeId2;
    int weight;
};

// This struct represents the data in a node. It has a unique id, and x, y coordinates.
// It also keeps track of connected edges of a node to avoid having to search through all
// edges to find one for a specific node. This also prevents the need for a separate
// collection for edges.
struct Node{
    int id;
    int x;
    int y;

    // Shared ptr since one edge can be owned by two nodes. This avoids duplicate edges existing and
    // handles memory management.
    std::vector<std::shared_ptr<Edge>> nodeEdges;
};


class MyGraph {
public:
    void AddNode(int id, int x, int y);
    void AddEdge(int firstNode, int secondNode, int weight);
    void ReadFile(const std::string& filename);
    void PrintNode(int id);

private:
    // It could happen that a node is added with a non-sequential ID. This makes checking if a node exists very
    // costly. An unordered map is used for this reason. For this datastructure the lookup time will not increase
    // by size. As elements are inserted in a pre-defined location by use of a hash function. And no comparing
    // of indices is needed.
    std::unordered_map<int, Node> nodes;
};


#endif //CODE_MYGRAPH_H
