// Copyright (C) 2023 Anton "optiklab" Yarkov
// https://github.com/optiklab/path-algorithms-in-a-graph
// See LICENSE file in the repo.
#pragma once
#ifndef Dijkstra_Queue_H
#define Dijkstra_Queue_H

#include <queue>
#include "pathFindingBase.h"

class dijkstraQueue : public pathFindingBase
{
private:
	vector<int> _queue;
	shared_ptr<vector<int>> _shortestPaths;

public:
	dijkstraQueue(shared_ptr<vector<int>> shortestPaths) : _shortestPaths(shortestPaths)
	{
	}

	virtual void insert(int node)
	{
		_queue.push_back(node);
	}

	/// <summary>
	/// Finds element with minimal distance from starting node.
	/// Returns this node, but also removes it from the queue.
	/// </summary>
	virtual int getFirst()
	{
		int minimum = INF;
		int minimumNode = -1;

		for (int i = 0; i < _queue.size(); i++)
		{
			int to = _queue[i];
			int newDistance = _shortestPaths->at(to);

			if (minimum > newDistance)
			{
				minimum = newDistance;
				minimumNode = to;
			}
		}
		
		if (minimumNode != -1)
		{
			remove(_queue.begin(), _queue.end(), minimumNode);
		}

		return minimumNode;
	}

	virtual bool isEmpty()
	{
		return _queue.empty();
	}
};

#endif