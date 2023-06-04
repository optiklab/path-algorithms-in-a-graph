# path-algorithms-in-a-graph

Answers on question why BFS, DFS, Dijkstra and A-Star algorithms are basically same algorithm.

Here you can find four data structures implemented in C++ for finding paths in a graph using four different algorithms:
- A queue for Breadth First Search
- A stack for Depth First Search
- A priority queue based on vector<T> for Dijkstra
- A priority queue based on vector<T> for A-Start 

# How to run


# Graph for testing

Program generates a graph of 25 (5 x 5 field) nodes, for test purpose.

For BFS and DFS algorithms it should be Zero-Weight Graph. For Dijkstra and A-Star algorithms it generates Non-Zero-Weight Graph.
You can keep using graph with weights for BFS and DFS as well, but it will not consider weights for finding a path. Thus, you will clearly see the applicability of each algorithm to certain tasks.

If you want to generate zero-weight graph, then you need to change variable to false:
bool generateWeights = true;

Basically, generate graph looks like this:
(0, 0) - (0, 1) - (0, 2) - (0, 3) - (0, 4)
   |        |        |        |        |
(1, 0) - (1, 1) - (1, 2) - (1, 3) - (1, 4)
   |        |        |        |        |
(2, 0) - (2, 1) - (2, 2) - (2, 3) - (2, 4)
   |        |        |        |        |
(3, 0) - (3, 1) - (3, 2) - (3, 3) - (3, 4)
   |        |        |        |        |
(4, 0) - (4, 1) - (4, 2) - (4, 3) - (4, 4)

where pairs are Row and Column numbers, or you can think of it as X and Y.

# Graph representation in memory

There are number of ways to represent graph in memory. This implementation uses Adjacency List of nodes with its Weights.

# Results

As a result of execution (if you didn't change the code) you might expect those results:

When you uncomment BFS Queue Data Structure (i.e. you execute BFS), then it finds this path:
(0, 0)
   | 
(1, 0) 
   |
(2, 0)
   |
(3, 0)
   | 
(4, 0) - (4, 1) - (4, 2) - (4, 3) - (4, 4)

Contrary, DFS finds path: 
(0, 0) - (0, 1) - (0, 2) - (0, 3) - (0, 4)
                                        |
                                    (1, 4)
                                       |
                                    (2, 4)
                                       |
                                    (3, 4)
                                       |
                                    (4, 4)

Dijkstra finds SHORTEST path with COST = 182:
       1        4        7       10 
(0, 0) - (0, 1) - (0, 2) - (0, 3) - (0, 4)
                                13     |
                                    (1, 4)
                                31     |
                                    (2, 4)
                                49     |
                                    (3, 4)
                                67     |
                                    (4, 4)
While coincidentally BFS finds same COST=182,it doesn't consider weights actually.
This is clearly seen in DFS, which finds COST=398 in the same time.

A-Star algorithm finds SHORTEST/QUICKIEST path with COST = 252: 
       1        4 
(0, 0) - (0, 1) - (0, 2)
                 8   |   23
                  (1, 2) - (1, 3)
                              |    28
                           (2, 3)
                              |    46
                           (3, 3)
                       64     |   78
                           (4, 3) - (4, 4)
						   
It's clear that A-Star is not for finding shortest path, but it's looking for most obvious path by using Euclidian metric in calculation of the minimal distance.

Based on this data I can clearly imagine types of tasks where I would apply one or another algorithm.

# Author

Copyright (C) 2023 Anton "optiklab" Yarkov

https://github.com/optiklab/path-algorithms-in-a-graph

See LICENSE file in the repo.