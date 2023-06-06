// Copyright (C) 2023 Anton "optiklab" Yarkov
// https://github.com/optiklab/path-algorithms-in-a-graph
// See LICENSE file in the repo.

#include <iostream>
#include "dfsStack.h"
#include "bfsQueue.h"
#include "aStarQueue.h"
#include "dijkstraQueue.h"

class FindAlgorithm
{
public:
    vector<int> FindPath(Graph& graph, int start, int finish, int finishX, int finishY)
    {
        int verticesNumber = graph.Nodes.size();

        vector<int> nodeColor(verticesNumber, WHITE);
        vector<int> shortestPath(verticesNumber, INF); // Current shortest path found from Start to i (INFinite from the beginning).
        vector<int> previousVertex(verticesNumber, -1); // Index of the vertex/node that is predecessor of i-th vertex in a shortest path to it.

        // We should use pointers here because we want to pass the pointer to a data-structure
        // so it may get all the updates automatically on every step.
        shared_ptr<vector<int>> ptrShortestPath = make_shared<vector<int>>(shortestPath);
        shared_ptr<Graph> ptrGraph = make_shared<Graph>(graph);

        ////////////////////////////////////////////////////////////////////////////////
        // TODO
        // UNCOMMENT DATA STRUCTURE YOU WANT TO USE:

        dfsStack customQueue;                                                   // UNCOMMENT TO USE DFS
        //bfsQueue customQueue;                                                 // UNCOMMENT TO USE BFS 
        //dijkstraQueue customQueue(ptrShortestPath);                           // UNCOMMENT TO USE DIJKSTRA
        //aStarQueue customQueue(finishX, finishY, ptrGraph, ptrShortestPath);  // UNCOMMENT TO USE A-STAR

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
                }
                else
                {
                    if (ptrShortestPath->at(to) > ptrShortestPath->at(current) + weight)
                    {
                        ptrShortestPath->at(to) = ptrShortestPath->at(current) + weight;
                    }
                }
            }

            nodeColor[current] = BLACK;
        }

        return {};
    }
};

/// <summary>
/// Generates graph connections and weights.
/// </summary>
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
                    if (generateWeights)
                    {
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
    // For BFS and DFS algorithms it should be Zero-Weight Graph. For Dijkstra and A* algorithms it should be Non-Zero-Weight Graph.
    bool generateWeights = true;
    createAdjacencyList(graph.Nodes, graph.Edges, generateWeights);

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
    // (0, 0) - (0, 1) - (0, 2) - (0, 3) - (0, 4)
    //                                         |
    //                                     (1, 4)
    //                                        |
    //                                     (2, 4)
    //                                        |
    //                                     (3, 4)
    //                                        |
    //                                     (4, 4)

    // DFS finds path: 
    // (0, 0)
    //    | 
    // (1, 0) 
    //    |
    // (2, 0)
    //    |
    // (3, 0)
    //    | 
    // (4, 0) - (4, 1) - (4, 2) - (4, 3) - (4, 4)

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
    // (BFS finds same COST=182 in the same time, but it doesn't look for COST actually)
    // (DFS finds COST=398 in the same time, since it doesn't look for COST)

    // A* finds SHORTEST/QUICKIEST path COST = 182:
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

    cout << "COST: " << cost << endl;

    return 0;
}