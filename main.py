import math

import pathPlanner
import generateRandomBoard
import csv
import numpy as np
import sys

aStarData = [None] * 4

fields = ["Facing Direction", "x Distance", "Y Distance", "Cost"]
rows = []

filename = "results.csv"


def reader(filename):
    data = np.loadtxt(filename, delimiter="\t", dtype=str)
    data = np.transpose(data)
    print(np.fliplr(np.rot90(np.array(data), 3)))

    newBoard = []
    for arr in data:
        newRow = []
        for val in arr:
            newRow.append(ord(val) - 48)
        newBoard.append(newRow)

    board = np.fliplr(np.rot90(np.array(newBoard), 3))

    return board


def getCost(current, next, board, moveType):
    cost = 0
    if moveType == "Bash":
        cost = 3
    elif moveType == "Forward":
        cost = math.ceil(board[next[1]][next[0]])
    elif moveType == "Right":
        cost = math.ceil(board[current[1]][current[0]] * .5)
    elif moveType == "Left":
        cost = math.ceil(board[current[1]][current[0]] * .5)

    if cost == 23 or cost == 18:  # turns goal into a terrain complexity of 1 and makes the turn on start = 1
        cost = 1
    return cost


def getDirection(moveType, currentDirection):
    direction = currentDirection
    if moveType == "Bash":
        pass
    elif moveType == "Forward":
        pass
    elif moveType == "Right":
        if currentDirection == 1:
            direction = 2
        elif currentDirection == 2:
            direction = 3
        elif currentDirection == 0:
            direction = 1
        elif currentDirection == 3:
            direction = 0

    elif moveType == "Left":
        if currentDirection == 1:
            direction = 0
        elif currentDirection == 3:
            direction = 2
        elif currentDirection == 2:
            direction = 1
        elif currentDirection == 0:
            direction = 3
    return direction


def write_to_csv(journey_storage_object, board_object):
    f = open("results.csv", 'a')
    board_copy = []
    for i in board_object:
        line = []
        for j in i:
            line.append(j)
        board_copy.append(line)
    # journey_storage_object[3].reverse()
    # journey_storage_object[0].reverse()
    # backtrack_array = [journey_storage_object[2]]
    path = journey_storage_object[0]
    goal_point = journey_storage_object[0][-1]
    # TODO write x dist = 0 y dist = 0 cost = final_cost
    direction = 0
    total_cost_minus = 100 - journey_storage_object[2]
    for i in range(0, len(path) - 1):
        xdist = abs(goal_point[0] - path[i][0])

        ydist = abs(goal_point[1] - path[i][1])

        cost = getCost(path[i], path[i + 1], board_copy, journey_storage_object[3][i])
        direction = getDirection(journey_storage_object[3][i], direction)
        print("Cost", cost, "xdist, ydist", xdist, ydist, "Direction", direction)
        rows.append([direction, xdist, ydist, total_cost_minus])
        total_cost_minus = total_cost_minus - cost
    for element in rows:
        appended_string = f"{element[0]}, {element[1]}, {element[2]}, {element[3]}\n"
        f.write(appended_string)


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    iterations = 1
    # numCol = 10
    # numRow = 10

    totalNodeCost = [0] * 8
    totalScore = [0] * 8

    filepath, filename, heuristic = sys.argv
    heuristic = int(heuristic)
    for x in range(iterations):
        board = reader(filename)  # Gives us a ndarray
        # np.array([s[0].astype(int) for s in board])
        # board = generateRandomBoard.getBoard(numCol, numRow) # Generating a random game board
        if iterations == 1:
            aStarData = pathPlanner.plan_path(board, heuristic)
            path = aStarData[0]
            totalNodeCost[heuristic] += aStarData[1]
            totalScore[heuristic] += aStarData[2]
            movesTaken = aStarData[3]

            # print("error in iteration ", x)
            print("")
            if iterations == 1:
                print("Path taken: ", path)
                print(movesTaken)

            # process = psutil.Process(os.getpid())
            # print(psutil.virtual_memory()[2])
            # print(process.memory_info().rss / (1024 * 1024), "MB")
            write_to_csv(aStarData, board)
        # Print our results
        print("Heuristic #", heuristic, ": ", "Total Nodes Expanded: ", totalNodeCost[heuristic],
              " Score: ",
              totalScore[heuristic])
