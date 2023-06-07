// Copyright (C) 2023 Anton "optiklab" Yarkov
// https://github.com/optiklab/path-algorithms-in-a-graph
// See LICENSE file in the repo.

#include <iostream>
#include "dfsStack.h"
#include "bfsQueue.h"
#include "dijkstraQueue.h"
#include "dijkstraPriorityQueue.h"
#include "aStarQueue.h"
#include "aStarPriorityQueue.h"

class FindAlgorithm
{
public:
    /// <summary>
    /// Universal algorithm to apply Path search using BFS, DFS, Dijkstra, A-Star.
    /// </summary>
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

        //dfsStack customQueue;                                                   // UNCOMMENT TO USE DFS
        //bfsQueue customQueue;                                                   // UNCOMMENT TO USE BFS 
        //dijkstraQueue customQueue(ptrShortestPath);                             // UNCOMMENT TO USE DIJKSTRA
        //dijkstraPriorityQueue customQueue(ptrShortestPath);                     // UNCOMMENT TO USE DIJKSTRA on priority queue
        //aStarQueue customQueue(finishX, finishY, ptrGraph, ptrShortestPath);    // UNCOMMENT TO USE A-STAR on vector
        aStarPriorityQueue customQueue(finishX, finishY, ptrGraph, ptrShortestPath);  // UNCOMMENT TO USE A-STAR on priority queue

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

    /// <summary>
    /// Finds path by using classic A-star approach (with no graph coloring).
    /// Finds a bit different path that also has lowest cost.
    /// </summary>
    vector<int> FindPathByClassicAStar(Graph& graph, int start, int finish, int finishX, int finishY)
    {
        int verticesNumber = graph.Nodes.size();

        vector<int> shortestPath(verticesNumber, INF); // длина кратчайшего пути от вершины s в i, сначала всегда равна бесконечности
        vector<int> previousVertex(verticesNumber, -1); // вершина, предшествующая i-й вершине на кратчайшем пути

        shortestPath[start] = 0;

        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> q;
        q.push({ 0, start });

        // Euclidian distance from node start to specified node id.
        auto calcEuristic = [](int x, int y, int x2, int y2) {
            return sqrt(
                pow(abs(
                    x2 > x ?
                    x2 - x :
                    x - x2), 2) +
                pow(abs(
                    y2 > y ?
                    y2 - y :
                    y - y2), 2));
        };

        while (!q.empty())
        {
            pair<int, int> cur = q.top();
            q.pop();

            int current = cur.second;

            if (current == finish)
            {
                // FOUND
                break;
            }

            for (int neighbourIndex = 0; neighbourIndex < graph.Edges[current].size(); neighbourIndex++)
            {
                int to = graph.Edges[current][neighbourIndex].first;
                int weight = graph.Edges[current][neighbourIndex].second;

                int newDistance = shortestPath[current] + weight;
                if (shortestPath[to] > newDistance)
                {
                    shortestPath[to] = newDistance;
                    previousVertex[to] = current;
                    int priority = shortestPath[to] + calcEuristic(graph.Nodes[to].X, graph.Nodes[to].Y, finishX, finishY);
                    q.push({ priority, to });
                }
            }
        }

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

bool firstFess(std::pair<int, int> lhs, int rhs) 
{
    return lhs.first < rhs;
}

int calculateCost(Graph& graph, vector<int> path)
{
    int cost = 0;
    for (int i = 1; i < path.size(); i++)
    {
        int from = path[i - 1];
        int to = path[i];
        int indexTo = std::lower_bound(graph.Edges[from].begin(), graph.Edges[from].end(), to, firstFess) - graph.Edges[from].begin();
        cost += graph.Edges[from][indexTo].second;
    }

    reverse(path.begin(), path.end());

    for (int i = 0; i < path.size(); i++)
    {
        cout << path[i] << " ";
    }

    cout << endl;

    return cost;
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

    // A* with universal algorithm finds SHORTEST/QUICKIEST path COST = 254 (it looks for QUICKIEST, not LOWEST COST):
    //        1        4 
    // (0, 0) - (0, 1) - (0, 2)
    //                  8   |
    //                   (1, 2)
    //                 24   |   41
    //                   (2, 2) - (2, 3)
    //                          46   |   63
    //                            (3, 3) - (3, 4)
    //                                        |   67
    //                                     (4, 4)

    // Classic A* finds SHORTEST&QUICKIEST path COST = 182:
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
    cout << "COST (universal): " << calculateCost(graph, algo.FindPath(graph, 0, 24, graph.Nodes[24].X, graph.Nodes[24].Y)) << endl;
    cout << "COST (a-star classic): " << calculateCost(graph, algo.FindPathByClassicAStar(graph, 0, 24, graph.Nodes[24].X, graph.Nodes[24].Y)) << endl;
 
    return 0;
}