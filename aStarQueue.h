#pragma once
#ifndef A_Star_Queue_H
#define A_Star_Queue_H

#include <queue>
#include <math.h>
#include "pathFindingBase.h"

using namespace std;

class aStarQueue : public pathFindingBase
{
private:
	vector<int> _queue;
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
	aStarQueue(int finishX, int finishY, shared_ptr<Graph> graph, shared_ptr<vector<int>> shortestPaths)
		:
		_shortestPaths(shortestPaths),
		_graph(graph)
	{
		_finishX = finishX;
		_finishY = finishY;
	}

	virtual void insert(int node)
	{
		_queue.push_back(node);
	}

	virtual int getFirst()
	{
		int minimum = INF;
		int minimumNode = -1;

		for (int i = 0; i < _queue.size(); i++)
		{
			int euristic = calcEuristic(_queue[i]);

			if (_shortestPaths->at(i) + euristic < minimum)
			{
				minimum = _shortestPaths->at(_queue[i]) + euristic;
				minimumNode = _queue[i];
			}
		}

		if (minimumNode != -1)
		{
			_queue.erase(std::lower_bound(_queue.begin(), _queue.end(), minimumNode));
		}

		return minimumNode;
	}

	virtual bool isEmpty()
	{
		return _queue.empty();
	}
};

#endif