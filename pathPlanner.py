#!/usr/bin/env python

import math
import random

import generateRandomBoard
from priority_queue import PriorityQueue
import numpy as np

aStarData = [None] * 3
def grid_to_index(mapdata, x, y):
    """
    Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
    :param x [int] The cell X coordinate.
    :param y [int] The cell Y coordinate.
    :return  [int] The index.
    """


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
    cols = len(mapdata[0])
    rows = len(mapdata)
    for i in range(len(addx)):
        if cols != y + addy[i] and \
                rows != x + addx[i] and \
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
    cols = len(mapdata[0])
    rows = len(mapdata)
    for i in range(len(addx)):
        if cols > y + addy[i] and \
                rows > x + addx[i] and \
                0 <= x + addx[i] and \
                0 <= y + addy[i]:
            neighbors.append((x + addx[i], y + addy[i]))

    return neighbors


def cleanup(path):
    """Cleans up the given path and returns the final path"""
    finalPath = []
    pathMoves = []
    prev_heading = 1
    for i in range(0,len(path)-1):
        nextHeading = 0
        finalPath.append(path[i])
        curr_pos = path[i]
        next_pos = path[i+1]
        if next_pos[1] / 2 - curr_pos[1] != 0:
            nextHeading = next_pos[1] - curr_pos[1] + 2
        if next_pos[0] / 2 - curr_pos[0] != 0:
            nextHeading = (((next_pos[0] - curr_pos[0]) + 3) % 3) * 2

        turn_cost = (4 + nextHeading - prev_heading) % 4

        if abs(next_pos[0]-curr_pos[0]) >= 2:
            pathMoves.append("Bash")
            holderpos = (next_pos[0],next_pos[1])
            finalPath.append(holderpos)
        else:
            pathMoves.append("Forward")
    return finalPath


def a_star(mapdata, start, goal, heuristicOption):
    """The start and goal are a tuple in grid format, mapdata is a matrix of size x,y"""
    ### REQUIRED CREDIT
    #print("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
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
            #print("Goal achieved")
            break

        #Add viable children to frontier
        #print("Currently: (%d,%d)" % (current[0], current[1]))
        for next in neighbors_of_4(mapdata,current[0], current[1]):
            try:
                cell_cost = mapdata[next[1]][next[0]] # Cost of the next cell
            except IndexError:
                cell_cost = math.inf


            if next[1]-current[1] != 0:
                nextHeading = next[1] - current[1] + 2
            if next[0]-current[0] != 0:
                nextHeading = (((next[0] - current[0])+3)%3)*2
            #print("%s Cost: %d Curr Heading = %d Next Heading = %d" % (next, mapdata[next[1]][next[0]], heading[current], nextHeading))

            try:
                turn_cost = (4+nextHeading-heading[current])%4 * int(math.ceil(float(mapdata[next[1]][next[0]])))
            except IndexError:
                turn_cost = math.inf

            #TODO: Heuristics go here:::
            verticleDistance = abs(goal[1]-next[1])
            horizontalDistance = abs(goal[0]-next[0])

            #print(horizontalDistance, verticleDistance, euclidean_distance(goal[0], next[0], goal[1], next[1]))
            if heuristicOption == 1: # No heuristic
                new_cost = cell_cost + cost_so_far[current] + turn_cost + random.randint(1,10)
            elif heuristicOption == 2: # Heuristic based upon the whichever is lower
                new_cost = cell_cost + cost_so_far[current] + turn_cost + min(verticleDistance, horizontalDistance) + random.randint(1,10)
            elif heuristicOption == 3: # Heuristic based upon whichever is higher
                new_cost = cell_cost + cost_so_far[current] + turn_cost + max(verticleDistance, horizontalDistance) + random.randint(1,10)
            elif heuristicOption == 4: # Heuristic where both are added together
                new_cost = cell_cost + cost_so_far[current] + turn_cost + horizontalDistance + verticleDistance
            elif heuristicOption == 5: # Heuristic that dominates 4 (the actual linear distance)
                new_cost = cell_cost + cost_so_far[current] + turn_cost + euclidean_distance(goal[0], goal[1], next[0], next[1])
            elif heuristicOption == 6: # Heuristic #5 multiplied by 3
                new_cost = cell_cost + cost_so_far[current] + turn_cost + (3 * euclidean_distance(goal[0], goal[1], next[0], next[1]))
            else:
                # If no valid heuristic is applied, error
                try:
                    raise Exception('ERROR: NO VALID HEURISTIC!!!!!')
                except Exception as error:
                    print(error)

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
            #print("%s Bash Cost: %d Curr Heading = %d Next Heading = %d" % (next, mapdata[next[1]][next[0]], heading[current], nextHeading))
            turn_cost = (4+nextHeading-heading[current])%4 * int(math.ceil(float(mapdata[next[1]][next[0]])))

            verticleDistance = abs(goal[1] - next[1])
            horizontalDistance = abs(goal[0] - next[0])

            # print(horizontalDistance, verticleDistance, euclidean_distance(goal[0], next[0], goal[1], next[1]))
            if heuristicOption == 1:  # No heuristic
                new_cost = cell_cost + cost_so_far[current] + turn_cost + random.randint(1,10)
            elif heuristicOption == 2:  # Heuristic based upon the whichever is lower
                new_cost = cell_cost + cost_so_far[current] + turn_cost + min(verticleDistance,horizontalDistance) + random.randint(1,10)
            elif heuristicOption == 3:  # Heuristic based upon whichever is higher
                new_cost = cell_cost + cost_so_far[current] + turn_cost + max(verticleDistance,horizontalDistance) + random.randint(1,10)
            elif heuristicOption == 4:  # Heuristic where both are added together
                new_cost = cell_cost + cost_so_far[current] + turn_cost + horizontalDistance + verticleDistance
            elif heuristicOption == 5:  # Heuristic that dominates 4 (the actual linear distance)
                new_cost = cell_cost + cost_so_far[current] + turn_cost + euclidean_distance(goal[0], goal[1], next[0], next[1])
            elif heuristicOption == 6:  # Heuristic #5 multiplied by 3
                new_cost = cell_cost + cost_so_far[current] + turn_cost + (
                            3 * euclidean_distance(goal[0], goal[1], next[0], next[1]))
            else:
                # If no valid heuristic is applied, error
                try:
                    raise Exception('ERROR: NO VALID HEURISTIC!!!!!')
                except Exception as error:
                    print(error)

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

    #print("Total Node Cost %d" % numNodes)
    aStarData[1] = len(expandedCells)

    #Making the path
    if came_from[goal] != None:
        while came_from[current] != None:
            path.append(current)
            current = came_from[current]
        path.append(start)
    else: #If the priority queue is empty, return an empty array
        return path

    #reverse path to make it so the first element is the start
    path = path[::-1]

    #print("A* completed")

    #Generated Total score
    totalScore = 97
    for point in path:
        totalScore -= mapdata[point[1],point[0]]
    #print(totalScore)
    aStarData[2]=totalScore
    return path


def plan_path(mapdata, heuristicOption):
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
    path = a_star(mapdata, start, goal, heuristicOption)
    finalPath = cleanup(path)

    aStarData[0] = finalPath

    ## Return a Path message
    return aStarData
