#!/usr/bin/env python

import math

import generateRandomBoard
from priority_queue import PriorityQueue
import numpy as np

def grid_to_index(mapdata, x, y):
    """
    Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
    :param x [int] The cell X coordinate.
    :param y [int] The cell Y coordinate.
    :return  [int] The index.
    """

    ### REQUIRED CREDIT
    return y * mapdata.info.width + x

def index_to_grid(mapdata, i):
    """
    Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
    :param x [int] The cell X coordinate.
    :param y [int] The cell Y coordinate.
    :return  [int] The index.
    """
    x = i%mapdata.info.width
    y = int(math.floor(i/mapdata.info.width))
    ### REQUIRED CREDIT
    return (x,y)


def euclidean_distance(x1, y1, x2, y2):
    """
    Calculates the Euclidean distance between two points.
    :param x1 [int or float] X coordinate of first point.
    :param y1 [int or float] Y coordinate of first point.
    :param x2 [int or float] X coordinate of second point.
    :param y2 [int or float] Y coordinate of second point.
    :return   [float]        The distance.
    """

    ### REQUIRED CREDIT
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def neighbors_of_4(mapdata, x, y):
    """
    Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
    :param mapdata [OccupancyGrid] The map information.
    :param x       [int]           The X coordinate in the grid.
    :param y       [int]           The Y coordinate in the grid.
    :return        [[(int,int)]]   A list of walkable 4-neighbors.
    """

    ### REQUIRED CREDIT
    neighbors = []

    addy = [1, -1, 0, 0]
    addx = [0, 0, 1, -1]
    for i in range(len(addx)):
        cols = generateRandomBoard.arrayCols
        rows = generateRandomBoard.arrayRows


        if cols - 1 != y + addy[i] and \
                rows - 1 != x + addx[i] and \
                -1 != x + addx[i] and \
                -1 != y + addy[i]:
            neighbors.append((x + addx[i], y + addy[i]))

    return neighbors

def neighbors_of_4_can_bash(mapdata, x, y):
    """
    Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
    :param mapdata [OccupancyGrid] The map information.
    :param x       [int]           The X coordinate in the grid.
    :param y       [int]           The Y coordinate in the grid.
    :return        [[(int,int)]]   A list of walkable 4-neighbors.
    """

    ### REQUIRED CREDIT
    neighbors = []

    addy = [2, -2, 0, 0]
    addx = [0, 0, 2, -2]
    for i in range(len(addx)):
        cols = generateRandomBoard.arrayCols
        rows = generateRandomBoard.arrayRows

        if cols - 1 > y + addy[i] and \
                rows - 1 > x + addx[i] and \
                0 <= x + addx[i] and \
                0 <= y + addy[i]:
            neighbors.append((x + addx[i], y + addy[i]))

    return neighbors



# @staticmethod
# def neighbors_of_8(mapdata, x, y):
#     """
#     Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
#     :param mapdata [OccupancyGrid] The map information.
#     :param x       [int]           The X coordinate in the grid.
#     :param y       [int]           The Y coordinate in the grid.
#     :return        [[(int,int)]]   A list of walkable 8-neighbors.
#     """
#
#     ### REQUIRED CREDIT
#     neighbors = []
#
#     addx = [1, -1, 0, 0, 1, 1, -1, -1]
#     addy = [0, 0, 1, -1, 1, -1, -1, 1]
#     for i in range(len(addx)):
#         try:
#            neighbors.append((x + addx[i], y + addy[i]))
#            break
#         except Exception as err:
#             print("Not a valid index!")
#     return neighbors

def cleanup(path):
    finalPath = []
    for i in range(0,len(path)-1):
        finalPath.append(path[i])
        curr_pos = path[i]
        next_pos = path[i+1]
        if abs(next_pos[0]-curr_pos[0]) >= 2:
            holderpos = (next_pos[0],next_pos[1])
            finalPath.append(holderpos)
    return finalPath


def a_star(mapdata, start, goal):
    """The start and goal are a tuple in grid format, mapdata is a matrix of size x,y"""
    ### REQUIRED CREDIT
    print("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    heading = {}
    came_from[start] = None
    cost_so_far[start] = 0
    heading[start] = 1 # 1 is up, 2 is to the right, 3 is bottom, 4 is to the left
    numNodes = 0
    nextHeading = 0

    came_from[goal] = None

    # Create the frontiers
    frontierStuff = frontier.get_queue()
    frontierCells = []
    for priorityTuple in frontierStuff:
        gridTuple = priorityTuple[1]
        frontierCells.append([gridTuple[0], gridTuple[1]])
        # print(gridTuple)


    #expanded cells to show pathed frontiers
    expandedCells = []


    #Go through graph until frontier is empty
    while not frontier.empty():
        current = frontier.get()
        # print(current)
        # update the frontiers visited just to let us see visually if we want
        expandedCells.append([current[0], current[1]])

        # If we have reached the goal, leave
        if current == goal:
            print("Goal achieved")
            break

        #Add viable children to frontier
        print("Currently: (%d,%d)" % (current[0], current[1]))
        for next in neighbors_of_4(mapdata,current[0], current[1]):
            cell_cost = mapdata[next[1]][next[0]] # Cost of the next cell

            if next[1]-current[1] != 0:
                nextHeading = next[1] - current[1] + 2
            if next[0]-current[0] != 0:
                nextHeading = (((next[0] - current[0])+3)%3)*2
            print("%s Cost: %d Curr Heading = %d Next Heading = %d" % (next, mapdata[next[1]][next[0]], heading[current], nextHeading))
            turn_cost = (4+nextHeading-heading[current])%4 * int(math.ceil(float(mapdata[next[1]][next[0]])))

            #TODO: Heuristics go here:::
            new_cost = cell_cost + cost_so_far[current] #+ euclidean_distance(current[0], current[1], next[0], next[1])

            if next not in cost_so_far or new_cost < cost_so_far[next]:
                numNodes+=1
                cost_so_far[next] = new_cost
                priority = new_cost # + PathPlanner.euclidean_distance(current[0], current[1], goal[0], goal[1])
                frontier.put(next, priority)

                # update frontier message
                frontierStuff = frontier.get_queue()
                frontierCells = []
                for priorityTuple in frontierStuff:
                    gridTuple = priorityTuple[1]
                    frontierCells.append([gridTuple[0], gridTuple[1]])

                #add parent to came_from table
                came_from[next] = current
                heading[next] = nextHeading


        #This is the code for the BASH functionality, so its only going straight I believe
        for next in neighbors_of_4_can_bash(mapdata,current[0], current[1]):
            cell_cost = 3 + mapdata[next[1]][next[0]] # Cost of the next cell plus the cell it just jumped over

            #Still have to turn to get in position to bash
            if next[1]/2-current[1] != 0:
                nextHeading = next[1] - current[1] + 2
            if next[0]/2-current[0] != 0:
                nextHeading = (((next[0] - current[0])+3)%3)*2
            print("%s Bash Cost: %d Curr Heading = %d Next Heading = %d" % (next, mapdata[next[1]][next[0]], heading[current], nextHeading))
            turn_cost = (4+nextHeading-heading[current])%4 * int(math.ceil(float(mapdata[next[1]][next[0]])))

            # TODO: Heuristics go here:::
            new_cost = cell_cost + cost_so_far[current] #+ euclidean_distance(current[0], current[1], next[0], next[1])

            if next not in cost_so_far or new_cost < cost_so_far[next]:
                numNodes+=1
                cost_so_far[next] = new_cost
                priority = new_cost # + PathPlanner.euclidean_distance(current[0], current[1], goal[0], goal[1])
                frontier.put(next, priority)

                # update frontier message
                frontierStuff = frontier.get_queue()
                frontierCells = []
                for priorityTuple in frontierStuff:
                    gridTuple = priorityTuple[1]
                    frontierCells.append([gridTuple[0], gridTuple[1]])

                #add parent to came_from table
                came_from[next] = current # This is currently skipping over one of the indexes
                heading[next] = nextHeading


    # Return the path found
    path = []
    current = goal

    print("Total Node Cost %d" % numNodes)

    if came_from[goal] != None:
        while came_from[current] != None:
            path.append(current)
            current = came_from[current]
        path.append(start)
    else:
        return path

    #reverse path to make it so the first element is the start
    path = path[::-1]
    print("A* completed")
    totalScore = 97
    for point in path:
        print(mapdata[point[1],point[0]])
        totalScore -= mapdata[point[1],point[0]]
    print(totalScore)
    return path


def plan_path(mapdata):
    """
    Plans a path between the start and goal locations in the requested.
    Internally uses A* to plan the optimal path.
    :param req
    """
    ## Request the map
    ## In case of error, return an empty path
    ## Execute A*
    start_y,start_x = np.where(mapdata==-2)
    # start = (1,2)
    start = (int(start_x),int(start_y))
    goal_y, goal_x = np.where(mapdata==-1)
    # goal = (3,4)
    goal = (int(goal_x),int(goal_y))
    path = a_star(mapdata, start, goal)
    finalPath = cleanup(path)
    ## Return a Path message
    return finalPath
