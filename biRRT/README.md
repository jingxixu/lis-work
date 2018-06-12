# RRT and Bi-directional RRT
Implementation of Rapidly-exploring random tree (RRT) algorithm and Bi-directionay RRT algorithm from scratch (without any collision checking packages), and visualize with matplotlib.

## Usage
Run the following command for detailed usage information

```python rrt.py -h``` 

__RRT__

```python rrt.py 40  world_obstacles.txt start_goal.txt ```

__BiRRT__

```python rrt.py 40  world_obstacles.txt start_goal.txt -b ```

__Command Line Parameters__
- `40` is the distance we want to take for each step; 
- `world_obstacles.txt` is the name of the text file containing  information about the obstacles in the map; ```start_goal.txt``` is the name of the text file containing information about the start and goal points; 
- `-b` is an optional parameter indicating whether RRT bi-directional is used. Use normal RRT by default.


## Results
- RRT

```python rrt.py 40  world_obstacles.txt start_goal.txt ```

- biRRT

```python rrt.py 40  world_obstacles.txt start_goal.txt -b ```


Our program can handle polygons of __ANY__ shape (not necessarily axis-parallel).


## Assumption
__Any special assumption we made are:__ 

1. The text files containing information about start, goal and obstacles must be in the same directory with ```rrt.py```.
2. All the points are integer points.
