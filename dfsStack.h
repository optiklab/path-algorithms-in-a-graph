// Copyright (C) 2023 Anton "optiklab" Yarkov
// https://github.com/optiklab/path-algorithms-in-a-graph
// See LICENSE file in the repo.
#pragma once
#ifndef Dfs_Stack_H
#define Dfs_Stack_H

#include <stack>
#include "pathFindingBase.h"

class dfsStack : public pathFindingBase
{
private:
	stack<int> _queue;

public:
	virtual void insert(int node)
	{
		_queue.push(node);
	}

	virtual int getFirst()
	{
		int value = _queue.top();
		_queue.pop();
		return value;
	}

	virtual bool isEmpty()
	{
		return _queue.empty();
	}
};

#endif