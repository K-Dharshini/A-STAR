<h1>ExpNo 4 : Implement A* search algorithm for a Graph</h1> 
<h3>Name: DHARSHINI K</h3>
<h3>Register Number: 212223230047</h3>

## Aim:
To Implement A * Search algorithm for a Graph using Python 3.

## Algorithm:
```
// A* Search Algorithm

1.  Initialize the open list

2.  Initialize the closed list
    put the starting node on the open 
    list (you can leave its f at zero)

3.  while the open list is not empty
    a) find the node with the least f on 
       the open list, call it "q"

    b) pop q off the open list
  
    c) generate q's 8 successors and set their 
       parents to q
   
    d) for each successor
        i) if successor is the goal, stop search
        
        ii) else, compute both g and h for successor
          successor.g = q.g + distance between 
                              successor and q
          successor.h = distance from goal to 
          successor (This can be done using many 
          ways, we will discuss three heuristics- 
          Manhattan, Diagonal and Euclidean 
          Heuristics)
          
          successor.f = successor.g + successor.h

        iii) if a node with the same position as 
            successor is in the OPEN list which has a 
           lower f than successor, skip this successor

        iV) if a node with the same position as 
            successor  is in the CLOSED list which has
            a lower f than successor, skip this successor
            otherwise, add  the node to the open list
     end (for loop)
  
    e) push q on the closed list
    end (while loop)
```

## Program:
```python
from collections import defaultdict

# Heuristic function: returns the estimated cost to reach the goal from the current node
def heuristic(node, H_dist):
    return H_dist.get(node, 0)

# Function to get neighbors of a node
def get_neighbors(node, Graph_nodes):
    return Graph_nodes.get(node, [])

# A* Algorithm function
def aStarAlgo(start_node, stop_node, Graph_nodes, H_dist):
    open_set = set([start_node])
    closed_set = set()
    g = {}  # Stores the actual distance from start_node to a node
    parents = {}  # Stores the parent of each node

    g[start_node] = 0
    parents[start_node] = start_node

    while open_set:
        n = None

        # Select node with the lowest f(n) = g(n) + h(n)
        for v in open_set:
            if n is None or g[v] + heuristic(v, H_dist) < g[n] + heuristic(n, H_dist):
                n = v

        if n is None:
            print("Path does not exist!")
            return None

        # If destination node is reached, reconstruct the path
        if n == stop_node:
            path = []
            while parents[n] != n:
                path.append(n)
                n = parents[n]
            path.append(start_node)
            path.reverse()
            print("Path found:", path)
            return path

        # Check neighbors of current node
        for (m, weight) in get_neighbors(n, Graph_nodes):
            if m not in open_set and m not in closed_set:
                open_set.add(m)
                parents[m] = n
                g[m] = g[n] + weight
            else:
                if g[m] > g[n] + weight:
                    g[m] = g[n] + weight
                    parents[m] = n
                    if m in closed_set:
                        closed_set.remove(m)
                        open_set.add(m)

        # Move current node from open to closed
        open_set.remove(n)
        closed_set.add(n)

    print("Path does not exist!")
    return None

# ---------------- Input Section ----------------
Graph_nodes = defaultdict(list)

# Input: Edges with weights
num_edges = int(input("Enter the number of edges: "))
print("Enter the edges in the format 'start end weight':")
for _ in range(num_edges):
    start, end, weight = input().split()
    weight = int(weight)
    Graph_nodes[start].append((end, weight))
    Graph_nodes[end].append((start, weight))  # Assuming undirected graph

# Input: Heuristic values
H_dist = {}
num_nodes = int(input("Enter the number of nodes: "))
print("Enter the heuristic values in the format 'node heuristic_value':")
for _ in range(num_nodes):
    node, h_value = input().split()
    H_dist[node] = int(h_value)

# Input: Start and goal nodes
start_node = input("Enter the start node: ")
stop_node = input("Enter the stop node: ")

# Run A* Algorithm
aStarAlgo(start_node, stop_node, Graph_nodes, H_dist)
```

<hr>
<h2>Sample Graph I</h2>
<hr>

![image](https://github.com/natsaravanan/19AI405FUNDAMENTALSOFARTIFICIALINTELLIGENCE/assets/87870499/b1377c3f-011a-4c0f-a843-516842ae056a)

<hr>
<h2>Sample Input</h2>
<hr>
10 14 <br>
A B 6 <br>
A F 3 <br>
B D 2 <br>
B C 3 <br>
C D 1 <br>
C E 5 <br>
D E 8 <br>
E I 5 <br>
E J 5 <br>
F G 1 <br>
G I 3 <br>
I J 3 <br>
F H 7 <br>
I H 2 <br>
A 10 <br>
B 8 <br>
C 5 <br>
D 7 <br>
E 3 <br>
F 6 <br>
G 5 <br>
H 3 <br>
I 1 <br>
J 0 <br>
<hr>
<h2>Sample Output</h2>
<hr>
Path found: ['A', 'F', 'G', 'I', 'J']


<hr>
<h2>Sample Graph II</h2>
<hr>

![image](https://github.com/natsaravanan/19AI405FUNDAMENTALSOFARTIFICIALINTELLIGENCE/assets/87870499/acbb09cb-ed39-48e5-a59b-2f8d61b978a3)


<hr>
<h2>Sample Input</h2>
<hr>
6 6 <br>
A B 2 <br>
B C 1 <br>
A E 3 <br>
B G 9 <br>
E D 6 <br>
D G 1 <br>
A 11 <br>
B 6 <br>
C 99 <br>
E 7 <br>
D 1 <br>
G 0 <br>
<hr>
<h2>Sample Output</h2>
<hr>
Path found: ['A', 'E', 'D', 'G']

## Result:
A* search algorithm for a Graph has been implemented.
