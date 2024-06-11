//
// Created by tomst on 8-6-2024.
//

#ifndef DATASTRUCTURES_ASTAR_H
#define DATASTRUCTURES_ASTAR_H

#include "MyGraph.h"
#include <unordered_map>

// Struct to keep track of specific information for the algorithm per node. To keep it seperated
// from the graph datastructure since it's specific for this algorithm.
struct NodeAlgorithmInfo {
    int nodeId = -1;
    int finalCost = -1;
    int distanceToEndNode = -1; // Distance between node and end node in manhattan distance
    int shortestPathCost = -1; // This includes weights of shortest paths nodes added
    int shortestPathPreviousNodeId = -1;
};

class AStar
{
public:
    AStar();
    void FillGraph(const std::string& filename);
    void FindPath(int nodeId1, int nodeId2);

private:
    // This method picks from unexploredNodesPool, checks neighbours, removes it from pool, resorts pool.
    int ExploreNewNode();
    std::vector<Node> FindNeighbours(Node node); // This method collects the neighbour node.
    NodeAlgorithmInfo CalculateFinalCost(const Node &originNode, const Node &destinationNode);
    int UpdateNodeAlgorithmInfo(const Node& node, const Node& previousNode, bool newNode);

    MyGraph graph;
    // I need to make sure that I do not use extra money for the unexplored pool. They need to point to the same
    // memory.
    std::vector<NodeAlgorithmInfo> unexploredNodesPool;
    // The collection of all the nodes that have been explored, or at least found.
    std::unordered_map<int, NodeAlgorithmInfo> exploredNodes;

    Node origin;
    Node destination;

    void SortExplorablePool();
    void PrintExplorablePool();

    void PrintShortestPath();
};


#endif //DATASTRUCTURES_ASTAR_H
