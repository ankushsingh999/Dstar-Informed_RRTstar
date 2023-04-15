# Dstar-Informed_RRTstar

Files are separated into two folders. 
For the **Informed RRT***, the **main.py** loads the map image **WPI_map.jpg** and calls classes and functions to run planning tasks. 
For the **D***,  the **main.py** loads two map, a static one and a dynamic one. The static one is for the initial planning, while the dynamic one tells where the new obstacles are. As you haven't written anything yet, a message should be returned stating no path is found
the coordinate system used here is **[row, col]**, which is different from [x, y] in Cartesian coordinates. In README and the code comment, when the word '**point**' is used, it refers to a simple list [row, col]. When the word '**node**' or '**vertex**' is used, it refers to either the Node class in RRT ,or a node/vertex in a graph in PRM.

# D*

Some of the key functions of D* algorithm are:

The d star algorithm is a variant of A* algorithm to find an optimal path between two points. The D* updates the path cost on the fly as it explores the graph, contrary to A* which needs complete search of the graph. 

D* has the ability to handle dynamic environments, where the graph or grid is changing.
1. The **run** function is the main function of D* algorithm, which includes two main steps. The first step is to search from goal to start in the static map. The second step is to move from start to goal. If any change detected in the second step, the cost should be updated and a new path should be replanned.
2. The **process_state** function that pops node from the open list, and process the node and its neighbors based on the state. 
3. The **prepare_repair** function that senses the neighbors of the given node and locate the new obstacle nodes for cost modification.
4. The **modify_cost** function that modifies the cost from the obstacle node and its neighbor, and put them in the Open list for future search.
5. The **repair_replan** that replans a path from the current node to the goal

Results:

Grid 1:

![ezgif com-gif-maker (1)](https://user-images.githubusercontent.com/64325043/231954222-7b1141fc-016c-4d3f-b2ff-825e013bb12e.gif)

Grid 2:

![ezgif com-gif-maker (2)](https://user-images.githubusercontent.com/64325043/231954262-1f5cc60b-f047-4b56-9d0f-bd12188aa345.gif)


Grid 3:

![ezgif com-gif-maker (3)](https://user-images.githubusercontent.com/64325043/231954293-86a0f702-9780-42a6-be50-a63e2b055b64.gif)

# Informed RRT*:

The informed RRT* is an extension of the RRT*. The algorithm tries to find an optimal path between start and goal nodes by generating a rapidly exploring random tree in the configuration space of the system. The tree grows until it reaches the goal. But in Informed RRT* the heuristic function guides the tree towards the goal region. The heuristic function is an ellipsoidal heuristic. The ellipsoidal heuristic guides the tree growth towards the goal.

Results :

![Informed_RRT](https://user-images.githubusercontent.com/64325043/231954601-ca61ce03-a9fc-46f6-9817-423e6ed7ed51.png)


