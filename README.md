# path-algorithms-in-a-graph

## Disclarimer

The project is a bit dirty and unfinished, since it was created for learning and testing some ideas.

The project is supported by the article [Universal implementation of BFS, DFS, Dijkstra and A-Star algorithms](https://github.com/optiklab/path-algorithms-in-a-graph/wiki).

## About

The project is supported by the article []().

Shows why BFS, DFS, Dijkstra and A-Star algorithms are basically same algorithm.

Here you can find four data structures implemented in C++ for finding paths in a graph using a universal path finding algorithm:
- A queue for **Breadth First Search**
- A stack for **Depth First Search**
- A slow queue based on vector<T> for **Dijkstra**
- A slow queue based on vector<T> for **A-Start**

## Important note

This code contains inefficient implementation of queueing mechanism for Dijkstra and A-Star as the aim of this project was to try out some ideas and to show that all four algorithms are very similar.

It is challenging to implement all algorithms efficiently using universal data structure, but it's interesting enough to explore. 

For more performance look at [my other project that implements those algorithms efficiently](https://github.com/optiklab/path-algorithms-on-a-grid-map) and uses more visual approach and much more test data for it.

## How to run

The project is written in C++ with STL use only.
Project file is built using Visual Studio 2022 and Microsoft Windows. So, you basically need to open the project using VS and press F5.

But if you need to run on linux there shouldn't be any problem to build using g++ or clang.

## Graph representation in memory

There are number of ways to represent graph in memory. This implementation uses Adjacency List of nodes with its Weights.

## Tests

### Test Case 1. Lazily generated graph

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

### Results

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

Contrary, BFS finds path (with COST=182 if you run it on weighed graph): 
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

### Test Case 2. Custom weights graph

Instead of generating the graph, let's try to create a custom weighed graph:
```bash
(0, 0) -1- (0, 1) -1- (0, 2) -1- (0, 3) -2- (0, 4)
   |          |          |          |         |
   2          1          1          2         2
   |          |          |          |         |
(1, 0) -2- (1, 1) -1- (1, 2) -2- (1, 3) -1- (1, 4)
   |          |          |          |         |
   2          1          1          1         1
   |          |          |          |         |
(2, 0) -1- (2, 1) -1- (2, 2) -1- (2, 3) -2- (2, 4)
   |          |          |          |         |
   2          1          1          1         2
   |          |          |          |         |
(3, 0) -2- (3, 1) -2- (3, 2) -1- (3, 3) -2- (3, 4)
   |          |          |          |         |
   2          1          1          2         2
   |          |          |          |         |
(4, 0) -2- (4, 1) -1- (4, 2) -2- (4, 3) -2- (4, 4)
```

DFS finds path 24<-23<-22<-21<-20<-15<-10<-5<-0 with COST = 15 (but it doesn't look for COST actually) : 
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

BFS finds other root 24<-19<-14<-9<-8<-7<-6<-1<-0 with the COST=11 (but again, it doesn't look for COST actually) : 
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

Dijkstra and A* finds SHORTEST path 24<-19<-18<-13<-12<-7<-6<-1<-0 with COST = 10:
```bash
(0, 0) -1- (0, 1)
             |
             1 
             |
           (1, 1) -1- (1, 2)
                         |
                         1 
                         |
                      (2, 2) -1- (2, 3)
                                    |
                                    1 
                                    |
                                  (3, 3) -1- (3, 4)
                                               |
                                               1 
                                               |
                                             (4, 4)
```

Fun enough, if we simply use euclidian logic in selecting the next node for traversal and don't consider weight at all, then
we take the quickiest path 24<-19<-18<-13<-12<-7<-2<-1<-0 in terms of euclidian distance and end up with QUICKIEST path of COST = 10 (it looks for QUICKIEST, not LOWEST COST):
```bash
(0, 0) -1- (0, 1) -1- (0, 2)
                         |
                         1 
                         |
                      (1, 2)
                         |
                         1 
                         |
                      (2, 2) -1- (2, 3)
                                    |
                                    1 
                                    |
                                  (3, 3) -1- (3, 4)
                                               |
                                               1 
                                               |
                                             (4, 4)
```

## Final words

It's very hard to  find a simple enough example of a graph that would show a big difference in Dijkstra and A-Star algorithms. So, please look at [my other project that implements those algorithms efficiently](https://github.com/optiklab/path-algorithms-on-a-grid-map) and uses more visual approach and much more test data for it.

The goal of this project was to show that all four algorithms are very similar. 

But unfortunately, it is hard to implement all four efficiently enough using universal data structure. 

If we really looking for performance (in 99% of cases) we shouldn't use universal approach.

## Author

Copyright (C) 2023 Anton "optiklab" Yarkov

https://github.com/optiklab/path-algorithms-in-a-graph

See LICENSE file in the repo.
