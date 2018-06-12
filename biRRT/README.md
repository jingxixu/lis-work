# Lab 5
Group 21 [jx2324, zz2517, ayz2013]

## Usage
You can find our main in ```rrt.py```. You can find our screenshot for part 1 in ```part1.png```. You can find our screenshot for part 2 in ```part2.png```.

For Part I:

```python rrt.py 40  world_obstacles.txt start_goal.txt ```

For Part II:

```python rrt.py 40  world_obstacles.txt start_goal.txt -b ```

Command Line Parameters:

```40``` is the distance we want to take for each step; ```world_obstacles.txt``` is the name of the text file containing 
information about the obstacles in the map; ```start_goal.txt``` is the name of the text file containing information about 
the start and goal points; ```-b``` is an optional parameter indicating whether RRT bi-directional is used.

Our program can handle polygons of __ANY__ shape (not necessarily axis-parallel).

## Implementation
__Our implementation specific details are:__ 

We start from the start position and expand the tree using the RRT algorithm 
as shown in the lecture. We first generate a random point ```rand_q``` in the map (we abandon the ```rand_q``` if it happens to be a node 
on the tree). We have a bias in our program, which means in one out of 5 ```rand_q```, we assign goal point ```goal``` to ```rand_q```. After we 
have found a valid ```rand_q```, we find the nearest nodes to this ```rand_q``` on the tree ```nearest``` and then find the new node to add to 
the tree called ```new_q``` by taking a step towards ```rand_q``` with the distance specified in the command line. However, before we actually make the step, 
we need to check if there exists a collision for this step. To do this, we call ```isFree(nearest, new_q, lines)```.

```lines``` is a collection of all boundaries of the obstacles. Each boundary in ```lines``` is defined by two points.
in ```isFree()``` function, we check if the line ```line0``` formed by two points ```nearest``` and ```new_q``` will intersect with each line
in ```lines```. To do this, we write a loop and call ```line_intersection(line0, lines[i])```. ```line_intersection()``` is a function we write
__FROM SCRATCH__ without using __ANY__ libraries, which returns the intersection point if two segments intesect and returns ```None``` if
the two segments do not intesect. Here, each segment is a straight line bounded by two points.

If ```isFree()``` returns ```True```, which indicates no collision if we take the step, we add the ```new_q``` into our tree and then 
repeat the process.

If the ```new_q``` we add to the tree has a distance to the ```goal``` less than that specified in the command line, we will use 
```isFree(new_q, goal, lines)``` to check if we can take the last step to reach ```goal```. If there is a collision in taking 
the last step, we need to find a new ```rand_q``` and keep growing the tree; if no collison is detected, we terminate the program and 
job is finished.

For the implementation of bi-directional RRT, we start by creating two trees rooted at start point and goal point respectively. The algorithm will generate a point ```rand_q``` in the map randomly and set it as a goal called ```new_goal```. To determine which tree to grow with respect to this ```new goal``` point, we calculate the Euclidean distance from ```new_goal``` to the nearest point on each tree, and we pick the tree with shorter distance. With the nearest node ```nearest```, we can determine a line towards ```new_goal``` and find a line segment such that the starting point is the nearest node and the length is the distance specified in the command line. The end point of this segment is called ```new_q```. To determine whether ```new_q``` can be added to the tree, we again call function ```is_Free(nearest, new_q, lines)```. If there is no collision, we grow the tree with ```new_q```; otherwise, we abandon this ```new_q```. We keep growing these two trees until they intersect.

## Assumption
__Any special assumption we made are:__ 

1. The text files containing information about start, goal and obstacles must be in the same directory with ```rrt.py```.
2. All the points are integer points.
