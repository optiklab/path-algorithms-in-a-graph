// Copyright (C) 2023 Anton "optiklab" Yarkov
// https://github.com/optiklab/path-algorithms-in-a-graph
// See LICENSE file in the repo.
#pragma once
#ifndef Path_Finding_Base_H
#define Path_Finding_Base_H

#include <vector>
#include <memory>

using namespace std;

const int INF = 1000000;

const int WHITE = 0;
const int GREY = 1;
const int BLACK = 2;

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