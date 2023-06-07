// Copyright (C) 2023 Anton "optiklab" Yarkov
// https://github.com/optiklab/path-algorithms-in-a-graph
// See LICENSE file in the repo.
#pragma once
#ifndef Dijkstra_PriorityQueue_H
#define Dijkstra_PriorityQueue_H

#include <queue>
#include "pathFindingBase.h"

class dijkstraPriorityQueue : public pathFindingBase
{
private:
	priority_queue<int, vector<int>, greater<int>> _queue;
	shared_ptr<vector<int>> _shortestPaths;

public:
	dijkstraPriorityQueue(shared_ptr<vector<int>> shortestPaths) : _shortestPaths(shortestPaths)
	{
	}

	virtual void insert(int node)
	{
		_queue.push(node);
	}

	/// <summary>
	/// Finds element with minimal distance from starting node.
	/// Returns this node, but also removes it from the queue.
	/// </summary>
	virtual int getFirst()
	{
		int current = _queue.top();
		_queue.pop();

		return current;
	}

	virtual bool isEmpty()
	{
		return _queue.empty();
	}
};

#endif