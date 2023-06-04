#include <iostream>
#include <numeric>
#include <queue>
#include <vector>
#include <string>
#include <stack>

#include "dfsStack.h"
#include "bfsQueue.h"
#include "aStarQueue.h"
#include "dijkstraQueue.h"

using namespace std;

const int WHITE = 0;
const int GREY = 1;
const int BLACK = 2;

class FindAlgorithm
{
public:
    vector<int> FindPath(Graph& graph, int start, int finish, int finishX, int finishY)
    {
        int verticesNumber = graph.Nodes.size();

        vector<int> nodeColor(verticesNumber, WHITE);
        vector<int> shortestPath(verticesNumber, INF); // длина кратчайшего пути от вершины s в i, сначала всегда равна бесконечности
        vector<int> previousVertex(verticesNumber, -1); // вершина, предшествующая i-й вершине на кратчайшем пути

        ////////////////////////////////////////////////////////////////////////////////
        // TODO
        // UNCOMMENT DATA STRUCTURE YOU WANT TO USE:
        //
        dfsStack customQueue;
        //bfsQueue customQueue;

        shared_ptr<vector<int>> ptrShortestPath = make_shared<vector<int>>(shortestPath);
        //dijkstraQueue customQueue(ptrShortestPath);

        shared_ptr<Graph> ptrGraph = make_shared<Graph>(graph);
        //aStarQueue customQueue(finishX, finishY, ptrGraph, ptrShortestPath);
        //
        // END OF TODO
        ////////////////////////////////////////////////////////////////////////////////

        customQueue.insert(start);
        nodeColor[start] = BLACK;
        ptrShortestPath->at(start) = 0;

        while (!customQueue.isEmpty())
        {
            int current = customQueue.getFirst();

            if (current == finish)
            {
                // Print path
                vector<int> path;

                int cur = finish;
                path.push_back(cur);

                while (previousVertex[cur] != -1)
                {
                    cur = previousVertex[cur];
                    path.push_back(cur);
                }

                reverse(path.begin(), path.end());

                return path;
            }

            for (int neighbourIndex = 0; neighbourIndex < graph.Edges[current].size(); neighbourIndex++)
            {
                int to = graph.Edges[current][neighbourIndex].first;
                int weight = graph.Edges[current][neighbourIndex].second;

                if (nodeColor[to] == WHITE)
                {
                    nodeColor[to] = GREY;
                    customQueue.insert(to);
                    previousVertex[to] = current;
                    ptrShortestPath->at(to) = ptrShortestPath->at(current) + weight;
                    //shortestPath[to] = shortestPath[current] + weight;
                }
                else
                {
                    if (ptrShortestPath->at(to) > ptrShortestPath->at(current) + weight)
                    {
                        ptrShortestPath->at(to) = ptrShortestPath->at(current) + weight;
                    }
                    //if (shortestPath[current] + weight < shortestPath[to])
                    //{
                    //    shortestPath[to] = shortestPath[current] + weight;
                    //}
                }
            }

            nodeColor[current] = BLACK;
        }

        return {};
    }
};

void createAdjacencyList(vector<GraphNode>& nodes, vector<vector<pair<int, int>>>& graph, bool generateWeights)
{
    vector<int> rowDirs{ 0, -1, 0, 1 };
    vector<int> colDirs{ -1, 0, 1, 0 };
    const int MAX_WEIGHT = 100;

    int weight = 1;

    for (int i = 0; i < nodes.size(); i++)
    {
        int rr = nodes[i].X;
        int cc = nodes[i].Y;

        vector<pair<int, int>> connection;
        for (int k = 0; k < 4; k++)
        {
            int row = rr + rowDirs[k];
            int col = cc + colDirs[k];

            if (row >= 0 && col >= 0 && row < 5 && col < 5)
            {
                int id = row * 5 + col;

                if (id != i)
                {
                    // If we need 33% probability of connection: uncomment these lines
                    //int dice = rand() % 3;
                    //if (dice == 0)
                    if (generateWeights)
                    {
                        //connection.push_back(make_pair(id, rand() % MAX_WEIGHT)); // Random
                        connection.push_back(make_pair(id, weight++));
                    }
                    else
                    {
                        connection.push_back(make_pair(id, 0));
                    }
                }
            }
        }
        graph.push_back(connection);
    }
}

bool first_less(std::pair<int, int> lhs, int rhs) 
{
    return lhs.first < rhs;
}

int main(int argc, char** argv)
{
    //vector<vector<int>> rooms{ {2147483647,-1,0,2147483647},{2147483647,2147483647,2147483647,-1},{2147483647,-1,2147483647,-1},{0,-1,2147483647,2147483647} };
    //int ans2 = longestSubarray({ 0,1,1,1,0,1,1,0,1 }); //5

    // Generate a graph of 25 (5 x 5 field) nodes, for test purpose.
    Graph graph;

    graph.Nodes.push_back({ 0, 0 }); // 1 (Index = 0)
    graph.Nodes.push_back({ 0, 1 });
    graph.Nodes.push_back({ 0, 2 });
    graph.Nodes.push_back({ 0, 3 });
    graph.Nodes.push_back({ 0, 4 }); // 5  (Index = 4)
    graph.Nodes.push_back({ 1, 0 });
    graph.Nodes.push_back({ 1, 1 });
    graph.Nodes.push_back({ 1, 2 });
    graph.Nodes.push_back({ 1, 3 });
    graph.Nodes.push_back({ 1, 4 }); // 10 (Index = 9)
    graph.Nodes.push_back({ 2, 0 });
    graph.Nodes.push_back({ 2, 1 });
    graph.Nodes.push_back({ 2, 2 });
    graph.Nodes.push_back({ 2, 3 });
    graph.Nodes.push_back({ 2, 4 }); // 15 (Index = 14)
    graph.Nodes.push_back({ 3, 0 });
    graph.Nodes.push_back({ 3, 1 });
    graph.Nodes.push_back({ 3, 2 });
    graph.Nodes.push_back({ 3, 3 });
    graph.Nodes.push_back({ 3, 4 }); // 20 (Index = 19)
    graph.Nodes.push_back({ 4, 0 });
    graph.Nodes.push_back({ 4, 1 });
    graph.Nodes.push_back({ 4, 2 });
    graph.Nodes.push_back({ 4, 3 });
    graph.Nodes.push_back({ 4, 4 }); // 25 (5x5) (Index = 24)

    // Now, create a graph representation. 
    // For BFS and DFS algorithms it should be Zero-Weight Graph:

    // createAdjacencyList(graph.Nodes, graph.Edges, false);

    // (0, 0) - (0, 1) - (0, 2) - (0, 3) - (0, 4)
    //    |        |        |        |        |
    // (1, 0) - (1, 1) - (1, 2) - (1, 3) - (1, 4)
    //    |        |        |        |        |
    // (2, 0) - (2, 1) - (2, 2) - (2, 3) - (2, 4)
    //    |        |        |        |        |
    // (3, 0) - (3, 1) - (3, 2) - (3, 3) - (3, 4)
    //    |        |        |        |        |
    // (4, 0) - (4, 1) - (4, 2) - (4, 3) - (4, 4)

    // BFS finds path:
    // (0, 0)
    //    | 
    // (1, 0) 
    //    |
    // (2, 0)
    //    |
    // (3, 0)
    //    | 
    // (4, 0) - (4, 1) - (4, 2) - (4, 3) - (4, 4)

    // DFS finds path: 
    // (0, 0) - (0, 1) - (0, 2) - (0, 3) - (0, 4)
    //                                         |
    //                                     (1, 4)
    //                                        |
    //                                     (2, 4)
    //                                        |
    //                                     (3, 4)
    //                                        |
    //                                     (4, 4)

    // For Dijkstra and A* algorithms it should be Non-Zero-Weight Graph:
    createAdjacencyList(graph.Nodes, graph.Edges, true);

    // Dijkstra finds SHORTEST path with COST = 182:
    //        1        4        7       10 
    // (0, 0) - (0, 1) - (0, 2) - (0, 3) - (0, 4)
    //                                 13     |
    //                                     (1, 4)
    //                                 31     |
    //                                     (2, 4)
    //                                 49     |
    //                                     (3, 4)
    //                                 67     |
    //                                     (4, 4)
    // (BFS finds same COST=182 in the same time)
    // (DFS finds COST=398 in the same time)

    // A* finds SHORTEST/QUICKIEST path with COST = 252: 
    //        1        4 
    // (0, 0) - (0, 1) - (0, 2)
    //                  8   |   23
    //                   (1, 2) - (1, 3)
    //                               |    28
    //                            (2, 3)
    //                               |    46
    //                            (3, 3)
    //                        64     |   78
    //                            (4, 3) - (4, 4)

    FindAlgorithm algo;
    vector<int> path = algo.FindPath(graph, 0, 24, graph.Nodes[24].X, graph.Nodes[24].Y);

    int cost = 0;
    for (int i = 1; i < path.size(); i++)
    {
        int from = path[i - 1];
        int to = path[i];
        int indexTo = std::lower_bound(graph.Edges[from].begin(), graph.Edges[from].end(), to, first_less) - graph.Edges[from].begin();
        cost += graph.Edges[from][indexTo].second;
    }

    return 0;
}