#include "MyGraph.h"
#include "AStar.h"

int main()
{
    AStar algorithm = AStar();
    algorithm.FillGraph("maps\\tiny.graph");
//    algorithm.FillGraph("maps\\USA-road-d.NY.graph");

    algorithm.FindPath(7, 15);
//    myGraph.AddNode(1, 1, 3);
//    myGraph.AddNode(2, 3, 5);
//    myGraph.AddNode(3, 4, 5);
//
//    myGraph.AddEdge(2, 2, 1, 30);
//    myGraph.AddEdge(1, 1, 2, 20);
//    myGraph.AddEdge(3, 1, 3, 20);
//    myGraph.AddEdge(4, 2, 3, 20);

//    myGraph.PrintNode(3);

    return 0;
}