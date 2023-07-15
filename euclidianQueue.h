// Copyright (C) 2023 Anton "optiklab" Yarkov
// https://github.com/optiklab/path-algorithms-in-a-graph
// See LICENSE file in the repo.
#pragma once
#ifndef Euclidian_H
#define Euclidian_H

#include <queue>
#include <math.h>
#include "pathFindingBase.h"

/// <summary>
/// An example of data structure that implements simple euclidian logic in selecting the next node for traversal.
/// 
/// It doesn't consider weight of the edges and simply takes the quickiest path in terms of euclidian distance.
/// </summary>
class euclidianQueue : public pathFindingBase
{
private:
	priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> _queue;
	shared_ptr<vector<int>> _shortestPaths;
	shared_ptr<Graph> _graph;
	int _finishX;
	int _finishY;

	/// <summary>
	/// Euclidian distance from node start to specified node id.
	/// </summary>
	int calcEuristic(int id)
	{
		return sqrt(
			pow(abs(
				_finishX > _graph->Nodes[id].X ?
				_finishX - _graph->Nodes[id].X :
				_graph->Nodes[id].X - _finishX), 2) +
			pow(abs(
				_finishY > _graph->Nodes[id].Y ?
				_finishY - _graph->Nodes[id].Y :
				_graph->Nodes[id].Y - _finishY), 2));
	}

public:
	euclidianQueue(int finishX, int finishY, shared_ptr<Graph> graph, shared_ptr<vector<int>> shortestPaths)
		:
		_shortestPaths(shortestPaths),
		_graph(graph)
	{
		_finishX = finishX;
		_finishY = finishY;
	}

	virtual void insert(int node)
	{
		int priority = _shortestPaths->at(node) + calcEuristic(node);

		_queue.push({ priority, node });
	}

	virtual int getFirst()
	{
		pair<int, int> current = _queue.top();
		_queue.pop();

		return current.second;
	}

	virtual bool isEmpty()
	{
		return _queue.empty();
	}
};

#endif