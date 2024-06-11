//
// Created by tomst on 8-6-2024.
//

#include "AStar.h"
#include <cmath>
#include <algorithm>

AStar::AStar()
{
    graph = MyGraph();
}

void AStar::FillGraph(const std::string& filename)
{
    graph.ReadFile(filename);
}

// This method starts the algorithm. It continues until the end node is found or there are no more nodes
void AStar::FindPath(int nodeId1, int nodeId2)
{
    // Get node1 and check if it exists.
    Node node1 = graph.GetNodes()[nodeId1];
    if (node1.id == -1)
    {
        std::cerr << "node1 not found!" << std::endl;
        return;
    }

    // Get node2 and check if it exists.
    Node node2 = graph.GetNodes()[nodeId2];
    if (node2.id == -1)
    {
        std::cerr << "node2 not found!" << std::endl;
        return;
    }

    origin = node1;
    destination = node2;

    // To calculate the distance to the end node we use the Manhattan distance, this saves computational
    // power, but still keeps the relative accuracy of distance despite the distances not being accurate.
    int differenceX = abs(node1.x - node2.x);
    int differenceY = abs(node1.y - node2.y);
    int distanceToEndNode = differenceX + differenceY;

    // Add the origin node to the unexplored nodes pool and the explored nodes pool. Also to the explored nodes pool
    // as all algorithm information about it is already known. And this info is needed in the algorithm to calculate
    // for neighbours when exploring a node.
    NodeAlgorithmInfo firstNodeInfo = {
            .nodeId = node1.id,
            .finalCost = distanceToEndNode,
            .distanceToEndNode = distanceToEndNode,
            .shortestPathCost = 0, // No weight for the first node.
            .shortestPathPreviousNodeId = -1 // This is the first node.
    };
    unexploredNodesPool.push_back(firstNodeInfo);
    exploredNodes[nodeId1] = firstNodeInfo;

    // Check if there is stuff in the graph.
    if (graph.GetNodes().empty())
    {
        std::cerr << "No nodes in the graph!" << std::endl;
        return;
    }

    std::cout << "Algorithm setup, starting with origin node: " << origin.id << " destination node: " << destination.id << std::endl;

    // Start the algorithm
    while (true)
    {
        // Step 1: Explore the neighbours of node, update the neighbours and put in explorable pool if applicable
        int result = ExploreNewNode();
        if (result == 0) // We found the final node.
            break;

        // Step 2: Sort explorable pool
        SortExplorablePool();
        PrintExplorablePool();
    }

    PrintShortestPath();
}

// Find the neighbouring nodes of a node. Keep in mind that the graph is directed.
std::vector<Node> AStar::FindNeighbours(Node node)
{
    std::vector<Node> neighbours;

    // Get the edges from the node.
    std::vector<std::shared_ptr<Edge>> edges = graph.GetNodes()[node.id].nodeEdges;
    for (const std::shared_ptr<Edge>& edge : edges)
    {
        // If the current node id is the 'from' of the edge, that means we are going in the right direction, as
        // the 'to' is the neighbouring node then.
        if (edge->from == node.id)
        {
            neighbours.push_back(graph.GetNodes()[edge->to]); // Add the neighbour node to the id.
        }
    }
    return neighbours;
}

// This method goes through the explorable pool and selects the first node, sorted on lowest cost.
// It first finds all neighbours, it does a check on all neighbours if this would be their shortest
// path, if so it would update that nodes information and place it back in the explorable pool.
// Then it sets this node as explored and removes it from the pool. If the current node is the
// destination node the algorithm is finished. If there are no more nodes in the explorable pool
// the algorithm ends.
int AStar::ExploreNewNode()
{
    // Select the first node in the pool.
    Node node = graph.GetNodes()[unexploredNodesPool[0].nodeId];

    // First check if this node is the end node. If so the destination has been explored and the algorithm is finished.
    if (node.id == destination.id)
    {
        return 0;
    }

    std::cout << "Exploring: " << node.id << std::endl;

    // Now check each neighbour of this node and update struct if needed.
    std::vector<Node> neighbours = FindNeighbours(node);
    for (const Node& neighbourNode : neighbours)
    {
        NodeAlgorithmInfo& currentAlgorithmInfo = exploredNodes[neighbourNode.id];

        // If id does not exist yet
        if (currentAlgorithmInfo.nodeId == -1)
        {
            // Update the information of the node that is useful for this algorithm.
            UpdateNodeAlgorithmInfo(neighbourNode, node, true);

            // Now add it to the explorable pool.
            unexploredNodesPool.push_back(exploredNodes[neighbourNode.id]);
        }
        else
        {
            // If the cost of this node to this neighbour is less than what it currently is that means
            // we add it to the explorable pool.
            if (UpdateNodeAlgorithmInfo(neighbourNode, node, false) == 1)
                unexploredNodesPool.push_back(exploredNodes[neighbourNode.id]);
        }
    }

    // Now remove the explored node from the unexplored pool, as it's been explored.
    unexploredNodesPool.erase(unexploredNodesPool.begin());

    return 1;
}

/**
 * This method calculates and returns NodeAlgorithmInfo of a node. The previousnode of the path is needed
 * for the calculation.
 * @param node  The node to be updates
 * @param previousNode  The node that is also needed for the calculations.
 */
NodeAlgorithmInfo AStar::CalculateFinalCost(const Node& node, const Node& previousNode)
{
    NodeAlgorithmInfo algorithmInfo;
    algorithmInfo.nodeId = node.id;

    // To calculate the shortestPathCost we use the previous nodes shortestPathCost plus the weight of the edge.
    int previousShortestPathCost = exploredNodes[previousNode.id].shortestPathCost;

    int weight; // For the edge we need to find the weight again.
    for (const std::shared_ptr<Edge>& edge : previousNode.nodeEdges)
    {
        if (edge->to == node.id) // This should always happen.
        {
            weight = edge->weight;
            break;
        }
    }

    algorithmInfo.shortestPathCost = previousShortestPathCost + weight;

    // Now the total cost is both added.
    algorithmInfo.finalCost = algorithmInfo.shortestPathCost + exploredNodes[node.id].distanceToEndNode;

    algorithmInfo.shortestPathPreviousNodeId = previousNode.id;

    return algorithmInfo;
}

/**
 * This method calculates and updates NodeAlgorithmInfo of a node. The previousnode of the path is needed
 * for the calculation.
 * @param node  The node to be updates
 * @param previousNode  The node that is also needed for the calculations.
 */
int AStar::UpdateNodeAlgorithmInfo(const Node& node, const Node& previousNode, bool newNode)
{
    // There are a few things that only need to be done once for a node.
    if (newNode)
    {
        exploredNodes[node.id].nodeId = node.id; // Set the id

        // To calculate the distance to the end node we use the Manhattan distance, this saves computational
        // power, but still keeps the relative accuracy of distance despite the distances not being accurate.
        int differenceX = abs(destination.x - node.x);
        int differenceY = abs(destination.y - node.y);
        exploredNodes[node.id].distanceToEndNode = differenceX + differenceY;
    }

    // Now calculate the costs for the previous node to the current node. If it's shorter or not set, update.
    NodeAlgorithmInfo calculatedAlgorithmInfo = CalculateFinalCost(node, previousNode);
    if (calculatedAlgorithmInfo.shortestPathCost < exploredNodes[node.id].shortestPathCost || newNode)
    {
        exploredNodes[node.id].shortestPathCost = calculatedAlgorithmInfo.shortestPathCost;
        exploredNodes[node.id].finalCost = calculatedAlgorithmInfo.finalCost;
        exploredNodes[node.id].shortestPathPreviousNodeId = calculatedAlgorithmInfo.shortestPathPreviousNodeId;
        return 1;
    }

    return 0;
}

void AStar::PrintExplorablePool()
{
    for (int i = 0; i < unexploredNodesPool.size(); i += 1)
    {
        std::cout << "pos: " << i << " id: " << unexploredNodesPool[i].nodeId << " cost: " << unexploredNodesPool[i].finalCost
        << " distance cost: " << unexploredNodesPool[i].distanceToEndNode << " path cost: " << unexploredNodesPool[i].shortestPathCost
        << " prev shortest node: " << unexploredNodesPool[i].shortestPathPreviousNodeId << std::endl;
    }
}

// We use std::sort on a lambda function to sort the vector of unexplored nodes on smallest cost.
// std::sort uses the lambda function to specify we want to sort on the finalCost variable of the struct.
void AStar::SortExplorablePool()
{
    std::sort(unexploredNodesPool.begin(), unexploredNodesPool.end(), [](const NodeAlgorithmInfo& a, const NodeAlgorithmInfo& b)
    {
        return a.finalCost < b.finalCost;
    });
}

void AStar::PrintShortestPath()
{
    // Get the destination nodes algorithm information and follow the trail of previous fastest nodes to get the
    // fastest path.
    std::cout << "Fastest Path: " << std::endl;
    std::vector<int> nodePathIds;
    int previousNodeId = destination.id;
    nodePathIds.push_back(previousNodeId);
    while (true)
    {
        previousNodeId = exploredNodes[previousNodeId].shortestPathPreviousNodeId;

        // If -1 that means we have found the origin.
        if (previousNodeId == -1)
        {
            break;
        }
        else
        {
            nodePathIds.push_back(previousNodeId);
        }
    }
    for (unsigned long long i = nodePathIds.size(); i > 0; i -= 1)
    {
        Node currentNode = graph.GetNodes()[nodePathIds[i - 1]];
        std::cout << "Node: " << currentNode.id << " X: " << currentNode.x << " Y: " << currentNode.y << std::endl;
    }
}
