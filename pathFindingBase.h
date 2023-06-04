#pragma once
#ifndef Path_Finding_Base_H
#define Path_Finding_Base_H

#include <memory>

using namespace std;

const int INF = 1000000;

// The node with its X, Y coordinates.
// Basically, this is only necessary for A* algorithm,
// since nor BFS or DFS or Dijkstra would consider it.
class GraphNode
{
public:
	int X;
	int Y;
};

class Graph
{
public:
	vector<vector<pair<int, int>>> Edges;
	vector<GraphNode> Nodes;
};

class pathFindingBase
{
public:
	virtual void insert(int node) = 0;

	virtual int getFirst() = 0;

	virtual bool isEmpty() = 0;
};

#endif