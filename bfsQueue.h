// Copyright (C) 2023 Anton "optiklab" Yarkov
// https://github.com/optiklab/path-algorithms-in-a-graph
// See LICENSE file in the repo.
#pragma once
#ifndef Bfs_Stack_H
#define Bfs_Stack_H

#include <queue>
#include "pathFindingBase.h"

class bfsQueue : public pathFindingBase
{
private:
	queue<int> _queue;

public:
	virtual void insert(int node)
	{
		_queue.push(node);
	}

	virtual int getFirst()
	{
		int value = _queue.front();
		_queue.pop();
		return value;
	}

	virtual bool isEmpty()
	{
		return _queue.empty();
	}
};

#endif