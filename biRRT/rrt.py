import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
import random, math

GRID_MIN = 0
GRID_MAX = 599

def dist(p1, p2):
     return math.sqrt((p1[0]-p2[0])**2 +(p1[1]-p2[1])**2)

def build_obstacle_course(obstacle_path, ax):
    vertices = list()
    codes = [Path.MOVETO]
    with open(obstacle_path) as f:
        quantity = int(f.readline())
        lines = 0
        for line in f:
            coordinates = tuple(map(int, line.strip().split(' ')))
            if len(coordinates) == 1:
                codes += [Path.MOVETO] + [Path.LINETO]*(coordinates[0]-1) + [Path.CLOSEPOLY]
                vertices.append((0,0)) #Always ignored by closepoly command
            else:
                vertices.append(coordinates)
    vertices.append((0,0))
    vertices = np.array(vertices, float)
    path = Path(vertices, codes)
    pathpatch = patches.PathPatch(path, facecolor='None', edgecolor='xkcd:violet')

    ax.add_patch(pathpatch)
    ax.set_title('Rapidly-exploring Random Tree')

    ax.dataLim.update_from_data_xy(vertices)
    ax.autoscale_view()
    ax.invert_yaxis()

    return path

def add_start_and_goal(start_goal_path, ax):
    start, goal = None, None
    with open(start_goal_path) as f:
        start = tuple(map(int, f.readline().strip().split(' ')))
        goal  = tuple(map(int, f.readline().strip().split(' ')))

    ax.add_patch(patches.Circle(start, facecolor='xkcd:bright green'))
    ax.add_patch(patches.Circle(goal, facecolor='xkcd:fuchsia'))

    return start, goal

def build_tree(start, goal, step, path, ax):
    back_tree = dict([(start, None)])
    lines = find_obstacles_lines(path)

    new_q, nearest, segment = start, start, None
    count = 0
    bias = 5
    while True:
        new_q, nearest, segment = step_one(None, step, back_tree, path,
                                           ax, lines) if count != 0 else step_one(goal, step, back_tree, path, ax, lines)

        count = (count + 1) % bias
        # print(dist(new_q, goal))


        back_tree.setdefault(new_q, nearest)

        pathpatch = patches.PathPatch(segment, edgecolor='xkcd:coral', linewidth=1)
        ax.add_patch(pathpatch)
        ax.add_patch(patches.Circle(new_q, facecolor='None', edgecolor='xkcd:mustard'))
        plt.pause(0.00001)

        # check if we can make the last step if we have entered into the target circle
        if dist(new_q, goal) <= step and isFree(new_q, goal, lines):
            break

    back_tree.setdefault(goal, new_q)
    codes = [Path.MOVETO, Path.LINETO]
    vertices = np.array([goal, new_q], float)
    segment = Path(vertices, codes)
    pathpatch = patches.PathPatch(segment, edgecolor='xkcd:coral', linewidth=1)
    ax.add_patch(pathpatch)
    ax.add_patch(patches.Circle(new_q, facecolor='None', edgecolor='xkcd:mustard'))
    plt.pause(0.00001)

    back_trace(back_tree, goal, start, ax, 'xkcd:royal')

def build_bidirectional_tree(start, goal, step, path, ax):
    lines = find_obstacles_lines(path)
    back_tree_start = dict([(start, None)])
    back_tree_goal  = dict([(goal, None)])

    new_q, nearest, segment = start, None, None
    new_goal, nearest2, segment2 = goal, goal, None
    tree = back_tree_start

    while segment is None:
        new_q, nearest, segment = step_one(None, step, back_tree_start,
                                           path, ax, lines)

    tree.setdefault(new_q, nearest)

    pathpatch = patches.PathPatch(segment, edgecolor='xkcd:coral', linewidth=1)
    ax.add_patch(pathpatch)
    ax.add_patch(patches.Circle(new_q, facecolor='None', edgecolor='xkcd:mustard'))
    plt.pause(0.00001)

    distance = dist(new_q, new_goal)
    while distance > step:

        tree = back_tree_start if tree == back_tree_goal else back_tree_goal
        new_goal = new_q
        segment = None
        if segment is None:
            new_q, nearest, segment = step_one(new_goal, step, tree,
                                                  path, ax, lines)

        bias = True
        while segment is None:
            new_q, nearest, segment = step_one(None, step, tree,
                                                  path, ax, lines)

        tree.setdefault(new_q, nearest)

        pathpatch = patches.PathPatch(segment, edgecolor='xkcd:coral', linewidth=1)
        ax.add_patch(pathpatch)
        ax.add_patch(patches.Circle(new_q, facecolor='None', edgecolor='xkcd:mustard'))
        plt.pause(0.00001)

        distance = dist(new_q, new_goal)

    # Last link
    tree.setdefault(new_goal, new_q)
    tree, origin = (back_tree_start, start) if tree == back_tree_goal else (back_tree_goal, goal)
    tree.setdefault(new_goal, new_q)
    codes = [Path.MOVETO, Path.LINETO]
    vertices = np.array([new_goal, new_q], float)
    segment = Path(vertices, codes)
    pathpatch = patches.PathPatch(segment, edgecolor='xkcd:coral', linewidth=1)
    ax.add_patch(pathpatch)
    ax.add_patch(patches.Circle(new_q, facecolor='None', edgecolor='xkcd:mustard'))
    plt.pause(0.00001)

    ax.add_patch(patches.Circle(new_goal, facecolor='xkcd:sky blue'))
    plt.pause(0.00001)

    back_trace(tree, new_goal, origin, ax, 'xkcd:royal')
    tree, origin = (back_tree_start, start) if tree == back_tree_goal else (back_tree_goal, goal)
    back_trace(tree, new_goal, origin, ax, 'xkcd:burnt siena')

def back_trace(child_parent_map, goal, start, ax, color):
    retrace = child_parent_map.get(goal)
    while retrace is not None:
        if retrace != start:
            ax.add_patch(patches.Circle(retrace, facecolor=color))
        retrace = child_parent_map.get(retrace)


def step_one(goal, step, tree, path, ax, lines):
    new_q = False
    while not new_q:
        rand_q = (random.randint(GRID_MIN, GRID_MAX),
                  random.randint(GRID_MIN, GRID_MAX))
        while rand_q in tree.keys():
            rand_q = (random.randint(GRID_MIN, GRID_MAX),
                    random.randint(GRID_MIN, GRID_MAX))
        if goal is not None:
            rand_q = goal

        nearest = min(tree.keys(), key=lambda n: dist(n, rand_q))
        u = np.array([1, 0])
        v = np.array([rand_q[0] - nearest[0], rand_q[1] - nearest[1]])

        # calculate the new point that needs to be added to the tree
        # norm of v cannot be 0 because rand_q cannot be points on the tree
        cos = np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))
        theta = np.arccos(cos)
        sin = np.sqrt(1 - cos**2)
        if rand_q[1] >= nearest[1]:
            new_q = tuple(map(int, (nearest[0]+step*cos, nearest[1]+step*sin)))
        else:
            new_q = tuple(map(int, (nearest[0]+step*cos, nearest[1]-step*sin)))

        if new_q[0] < GRID_MIN or new_q[0] > GRID_MAX or new_q[1] < GRID_MIN or new_q[1] > GRID_MAX:
            new_q = False
            continue
            
        if not isFree(nearest, new_q, lines):
            new_q = False
            if goal is not None:
                goal = None
            continue

        codes = [Path.MOVETO, Path.LINETO]
        vertices = np.array([nearest, new_q], float)
        segment = Path(vertices, codes)

    return new_q, nearest, segment

def find_obstacles_lines(path):
    """
    pass path to the function
    """  
    # find coordinates of each obstacle
    path.codes[0] = max(path.codes)
    bool_split = path.codes < max(path.codes)
    Indices = [i for i, x in enumerate(bool_split) if x == False]
    obstacles = list()
    for i in range(len(Indices)-1):
        obstacles.append(path.vertices[Indices[i]+1: Indices[i+1]])
    
    # generate lines of each obstacle
    lines = []
    
    for each_obstacle in obstacles:
        for i in range(len(each_obstacle)):
            pt1 = each_obstacle[i-1]
            pt2 = each_obstacle[i]
            
            lines.append([pt1, pt2])
            #lines.append(np.concatenate([pt1, pt2], axis = 0))
    return lines

def isFree(nearest, new_q, lines):
    line0 = [nearest, new_q]
    for i in range(len(lines)):
        if line_intersection(line0, lines[i]) == None:
            pass
        else:
            # print(line_intersection(line0, lines[i]))
            return False 
    return True

def line_intersection(line1, line2):
    I_x = [max(min(line1[0][0], line1[1][0]), min(line2[0][0], line2[1][0])), 
            min(max(line1[0][0], line1[1][0]), max(line2[0][0],line2[1][0]))]
    # print(I_x)
    if I_x[0] > I_x[1]:
        return None
    I_y = [max(min(line1[0][1], line1[1][1]), min(line2[0][1], line2[1][1])), 
            min(max(line1[0][1], line1[1][1]), max(line2[0][1],line2[1][1]))]
    # print(I_y)
    if I_y[0] > I_y[1]:
        return None
    if line1[0][0] != line1[1][0] and line2[0][0] != line2[1][0]:
        k1 = (line1[0][1]-line1[1][1]) / (line1[0][0]-line1[1][0])
        k2 = (line2[0][1]-line2[1][1]) / (line2[0][0]-line2[1][0])
        b1 = line1[0][1]-k1*line1[0][0]
        b2 = line2[0][1]-k2*line2[0][0]

        if k1 == k2:
            if b1 == b2:
                return (10000, 10000)
            else:
                return None
        x_intersection = round((b2 - b1) / (k1 - k2))
        if x_intersection < I_x[0] or x_intersection > I_x[1]:
            return None
        y_intersection = k1 * x_intersection + b1
        return (x_intersection, y_intersection)
    # line1 is vertical
    if line1[0][0] == line1[1][0] and line2[0][0] != line2[1][0]:
        k2 = (line2[0][1]-line2[1][1]) / (line2[0][0]-line2[1][0])
        b2 = line2[0][1]-k2*line2[0][0]
        x_intersection = line1[0][0]
        if x_intersection < I_x[0] or x_intersection > I_x[1]:
            return None
        y_intersection = k2 * x_intersection + b2
        if y_intersection < I_y[0] or y_intersection > I_y[1]:
            return None
        return (x_intersection, y_intersection)
    # line2 is vertical
    if line2[0][0] == line2[1][0] and line1[0][0] != line1[1][0]:
        k1 = (line1[0][1]-line1[1][1]) / (line1[0][0]-line1[1][0])
        b1 = line1[0][1]-k1*line1[0][0]
        x_intersection = line2[0][0]
        if x_intersection < I_x[0] or x_intersection > I_x[1]:
            return None
        y_intersection = k1 * x_intersection + b1
        if y_intersection < I_y[0] or y_intersection > I_y[1]:
            return None
        return (x_intersection, y_intersection)
    if line1[0][0] == line1[1][0] and line2[0][0] == line2[1][0]:
        if I_x[0] == I_x[1] and I_y[0] <= I_y[1]:
            return (10000, 10000)
        return None

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('distance', type=int,
                        help="Distance grown with each iteration")
    parser.add_argument('obstacle_path',
                        help="File path for obstacle set")
    parser.add_argument('start_goal_path',
                        help="File path for obstacle set")
    parser.add_argument('-b', '--bidirectional_tree', action='store_true',
                        help="Default is biased RRT, with this flag set is bidirectional RRT")
    args = parser.parse_args()

    fig, ax = plt.subplots()
    path = build_obstacle_course(args.obstacle_path, ax)
    start, goal = add_start_and_goal(args.start_goal_path, ax)

    plt.ion()

    codes = [Path.MOVETO] + [Path.LINETO]*3 + [Path.CLOSEPOLY]
    vertices = np.array([(1, 1), (1,3), (3, 3), (3,1), (0,0)], float)
    segment = Path(vertices, codes)

    # codes = [Path.MOVETO, Path.LINETO]
    # vertices = np.array([(0, 1), (2, 0)], float)
    # segment2 = Path(vertices, codes)

    #ax.add_patch(patches.PathPatch(segment))
    # ax.add_patch(patches.PathPatch(segment2))

    #print(segment.contains_point((0,0)))

    build_method = build_tree if not args.bidirectional_tree else build_bidirectional_tree
    build_method(start, goal, args.distance, path, ax)

    plt.ioff()
    plt.show()
