//
// Created by tomst on 2-6-2024.
//

#include "MyGraph.h"
#include <fstream>
#include <sstream>

// This functions adds a node to the graph.
void MyGraph::AddNode(int id, int x, int y)
{
    // Check if the node already exists by finding checking on key. If nodes.end that means it hasn't been found.
    if (nodes.find(id) == nodes.end())
    {
        Node newNode{.id= id,
                .x = x,
                .y = y};

        this->nodes.insert({id, newNode});
    }
    else
        std::cerr << "This shouldn't happen, an attempt was made to add a node that already exists!" << std::endl;
}

// This function adds an edge to nodes.
// Seems that two edges can be added to the same two nodes, examples:
//e 73346 73351 1114
//e 73351 73346 1114
// Hoe gaan we daar mee om?
void MyGraph::AddEdge(int firstNode, int secondNode, int weight)
{
    // First check if nodeId1 and nodeId2 exist. They are both required to add the edges to these nodes.
    if (nodes.find(firstNode) == nodes.end() || nodes.find(secondNode) == nodes.end())
    {
        std::cerr << "When adding an edge either one or both nodes did not exist!" << std::endl;
        return;
    }

    // Now check if the connection between the two nodes already exists. (undirected graph)
    for (int i = 0; i < nodes[firstNode].nodeEdges.size(); i += 1)
    {
        // Check if nodeId2 exists in edges connected to nodeId1. In it's nodeId1 or nodeId2 spot, both are possible
        // depending on the order of adding the nodes to the edge.
        Edge currentEdge = *nodes[firstNode].nodeEdges[i];
        if (currentEdge.nodeId2 == secondNode || currentEdge.nodeId1 == secondNode)
        {
            std::cerr << "When adding an edge between two nodes, there was already an edge between them!" << "First Node: " <<  firstNode << " Second Node: " << secondNode << std::endl;
            return;
        }
    }

    // Create the new edge
    Edge newEdge{.nodeId1 = firstNode,
                 .nodeId2 = secondNode,
                 .weight = weight};

    std::shared_ptr edge = std::make_shared<Edge>(newEdge);

    // Now add the edge to both nodes.
    nodes[firstNode].nodeEdges.push_back(edge);
    nodes[secondNode].nodeEdges.push_back(edge);
}

void MyGraph::PrintNode(int id)
{
    if (nodes.find(id) != nodes.end())
    {
        std::cout << "Printing info about node: " << id << "\tx: " << nodes[id].x << " y: " << nodes[id].y << std::endl << std::endl;

        // For every connected edge print information.
        for (int i = 0; i < nodes[id].nodeEdges.size(); i += 1)
        {
            Edge currentEdge = *nodes[id].nodeEdges[i];
            Node edgeOtherNode; // The other node that the edge connects to.
            if (nodes[id].nodeEdges[i]->nodeId1 == id)
            {
                edgeOtherNode = nodes[currentEdge.nodeId2];
            }
            else
            {
                edgeOtherNode = nodes[currentEdge.nodeId1];
            }

            std::cout << "Edge " << i + 1 << ":\t weight: " << currentEdge.weight << std::endl;
            std::cout << "Connected With Node - id: " << edgeOtherNode.id << " x: " << edgeOtherNode.x << " y: " << edgeOtherNode.y << std::endl << std::endl;
        }
    }
    else
    {
        std::cerr << "Node to print not found!" << std::endl;
    }
}

// Just an out-of-place function to split a string.
std::vector<std::string> split(const std::string& line) {
    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::string token;
    while (std::getline(iss, token, ' ')) {
        tokens.push_back(token);
    }
    return tokens;
}

// This function reads a file and parses it adding its contents to the   graph.
void MyGraph::ReadFile(const std::string& filename)
{
    std::ifstream file;
    file.open(filename); // Open file.
    if (!file.is_open())
    {
        std::cerr << "Unable to open file" << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::vector<std::string> splitWords = split(line);
        if (splitWords[0][0] == 'v') // A node
        {
            int nodeId = std::stoi(splitWords[1]);
            int x = std::stoi(splitWords[2]);
            int y = std::stoi(splitWords[3]);
            AddNode(nodeId, x, y);
        }
        else // An edge
        {
            int nodeId1 = std::stoi(splitWords[1]);
            int nodeId2 = std::stoi(splitWords[2]);
            int weight = std::stoi(splitWords[3]);
            AddEdge(nodeId1, nodeId2, weight);
        }
    }
    file.close();
}
