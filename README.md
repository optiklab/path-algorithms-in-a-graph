# path-algorithms-in-a-graph

Answers on question why BFS, DFS, Dijkstra and A-Star algorithms are basically same algorithm.

Here you can find four data structures implemented in C++ for finding paths in a graph using four different algorithms:
- A queue for **Breadth First Search**
- A stack for **Depth First Search**
- A priority queue based on vector<T> for **Dijkstra**
- A priority queue based on vector<T> for **A-Start**

# Important note

This code contains inefficient implementation of queueing mechanism for Dijkstra and A-Star as the aim of this project was to show that all four algorithms are very similar.

Unfortunately, it is hard to implement all four efficiently enough using universal data structure. If we really looking for performance (in 99% of cases) we shouldn't use universal approach.

Look at [my other project that implements those algorithms efficiently](https://github.com/optiklab/path-algorithms-on-a-grid-map) and uses more visual approach and much more test data for it.

# How to run

The project is written in C++ with STL use only.
Project file is built using Visual Studio 2022 and Microsoft Windows. So, you basically need to open the project using VS and press F5.

But if you need to run on linux there shouldn't be any problem to build using g++ or clang.

# Graph for testing

Program generates a graph of 25 (5 x 5 field) nodes, for test purpose.

```bash
( 0  ) - ( 1  ) - ( 2  ) - ( 3  ) - ( 4  )
  |        |        |        |        |
( 5  ) - ( 6  ) - ( 7  ) - ( 8  ) - ( 9  )
  |        |        |        |        |
( 10 ) - ( 11 ) - ( 12 ) - ( 13 ) - ( 14 )
  |        |        |        |        |
( 15 ) - ( 16 ) - ( 17 ) - ( 18 ) - ( 19 )
  |        |        |        |        |
( 20 ) - ( 21 ) - ( 22 ) - ( 23 ) - ( 24 )
```

For BFS and DFS algorithms it should be Zero-Weight Graph. For Dijkstra and A-Star algorithms it generates Non-Zero-Weight Graph.
You can keep using graph with weights for BFS and DFS as well, but it will not consider weights for finding a path. Thus, you will clearly see the applicability of each algorithm to certain tasks.

If you want to generate zero-weight graph, then you need to change variable to false:
```bash
bool generateWeights = true;
```

If to visualize the graph, it would look like this:
```bash
(0, 0) - (0, 1) - (0, 2) - (0, 3) - (0, 4)
   |        |        |        |        |
(1, 0) - (1, 1) - (1, 2) - (1, 3) - (1, 4)
   |        |        |        |        |
(2, 0) - (2, 1) - (2, 2) - (2, 3) - (2, 4)
   |        |        |        |        |
(3, 0) - (3, 1) - (3, 2) - (3, 3) - (3, 4)
   |        |        |        |        |
(4, 0) - (4, 1) - (4, 2) - (4, 3) - (4, 4)
```

where in brackets pairs are Row and Column numbers, or you can think of it as X and Y. Weights generated in both directions, but they are different, meaning that 0->1 has weight 1, while 1->0 has weight 3.

# Graph representation in memory

There are number of ways to represent graph in memory. This implementation uses Adjacency List of nodes with its Weights.

# Results

As a result of execution (if you didn't change the code) you might expect those results:

When you uncomment DFS Queue Data Structure (i.e. you execute DFS), then it finds this path:
```bash
(0, 0)
   | 
(1, 0) 
   |
(2, 0)
   |
(3, 0)
   | 
(4, 0) - (4, 1) - (4, 2) - (4, 3) - (4, 4)
```

The COST of this path is 398, but remember: nor DFS or BFS consider weights actually. They just traverse the graph with the hope to find a path at some point.

Contrary, BFS finds path (with COST=182 if you run it on weighten graph): 
```bash
(0, 0) - (0, 1) - (0, 2) - (0, 3) - (0, 4)
                                        |
                                    (1, 4)
                                       |
                                    (2, 4)
                                       |
                                    (3, 4)
                                       |
                                    (4, 4)
```

Both Dijkstra and A-Start find SHORTEST path with COST = 182 considering the lowest cost:
```bash
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
```

Fun enough, if we simply use euclidian logic in selecting the next node for traversal and don't consider weight at all, then
we take the quickiest path in terms of euclidian distance and end up with QUICKIEST path of COST = 254 (it looks for QUICKIEST, not LOWEST COST):
```bash
       1        4 
(0, 0) - (0, 1) - (0, 2)
                 8   |
                  (1, 2)
                24   |   41
                  (2, 2) - (2, 3)
                         46   |   63
                           (3, 3) - (3, 4)
                                       |   67
                                    (4, 4)
```

It's very hard to  find a simple enough example of a graph that would show a big difference in Dijkstra and A-Star algorithms. So, please look at [my other project that implements those algorithms efficiently](https://github.com/optiklab/path-algorithms-on-a-grid-map) and uses more visual approach and much more test data for it.

The goal of this project was to show that all four algorithms are very similar. 

But unfortunately, it is hard to implement all four efficiently enough using universal data structure. 

If we really looking for performance (in 99% of cases) we shouldn't use universal approach.

# Author

Copyright (C) 2023 Anton "optiklab" Yarkov

https://github.com/optiklab/path-algorithms-in-a-graph

See LICENSE file in the repo.