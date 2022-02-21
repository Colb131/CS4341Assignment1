import math
from time import time
import pandas as pd
import main
import pathPlanner
import generateRandomBoard
import csv
import numpy as np

aStarData = [None] * 4

fields = ["Facing Direction", "x Distance", "Y Distance", "Cost"]
rows = []

filename = "results.csv"

def reader():
    file = input("Please enter file name: ")

    data = np.loadtxt(file, delimiter="\t", dtype=str)
    data = np.transpose(data)
    print(np.fliplr(np.rot90(np.array(data), 3)))
    global heuristicOption
    heuristicOption = int(input("Please enter desired heuristic: "))

    newBoard = []
    for arr in data:
        newRow = []
        for val in arr:
            newRow.append(ord(val) - 48)
        newBoard.append(newRow)

    board = np.fliplr(np.rot90(np.array(newBoard), 3))


    return board

def getCost(current, next, board, moveType, totalCost):
    cost = totalCost
    if moveType == "Bash":
        if cost <= 0:
            cost = totalCost + 3
        else:
            cost = totalCost - 3
    elif moveType == "Forward":
        if cost <= 0:
            cost = totalCost + math.ceil(board[next[0]][next[1]])
        else:
            cost = totalCost - math.ceil(board[next[0]][next[1]])
    elif moveType == "Right":
        if cost <= 0:
            cost = totalCost + math.ceil(board[current[0]][current[1]] * .5)
        else:
            cost = totalCost - math.ceil(board[current[0]][current[1]] * .5)
    elif moveType == "Left":
        if cost <= 0:
            cost = totalCost + math.ceil(board[current[0]][current[1]] * .5)
        else:
            cost = totalCost - math.ceil(board[current[0]][current[1]] * .5)
    return cost


def getDirection(current, next, moveType, currentDirection):
    direction = currentDirection
    if moveType == "Bash":
        pass
    elif moveType == "Forward":
        pass
    elif moveType == "Right":
        if currentDirection == 1:
            direction = -2
        elif currentDirection == -1:
            direction = 2
        elif currentDirection == 2:
            direction = 1
        elif currentDirection == -2:
            direction = -1

    elif moveType == "Left":
        if currentDirection == 1:
            direction = 2
        elif currentDirection == -1:
            direction = -2
        elif currentDirection == 2:
            direction = -1
        elif currentDirection == -2:
            direction = 1
    return direction


def write_to_csv(journey_storage_object, gameboard):
    global rows
    board_copy = gameboard
    print(board_copy)
    # journey_storage_object[3].reverse()
    # journey_storage_object[0].reverse()
    # backtrack_array = [journey_storage_object[2]]
    path = journey_storage_object[0]
    goal_point = journey_storage_object[0][-1]
    # TODO write x dist = 0 y dist = 0 cost = final_cost
    startDirection = 2

    orginalCost = journey_storage_object[2] - 1
    start_point = journey_storage_object[0][0]
    #add start
    xd = abs(goal_point[0] - start_point[0])
    yd = abs(goal_point[1] - start_point[1])
    print("Moves Taken", journey_storage_object[3])
    print(len(path), len(journey_storage_object[4]))
    print(path)
    print("Cost", orginalCost, "xdist, ydist", xd, yd, "Direction", 1)
    orginalDirection = 1
    rows.append([1, xd, yd, orginalCost])
    for i in range(1, len(journey_storage_object[3])):
        xdist = abs(goal_point[0] - path[i+1][0])

        ydist = abs(goal_point[1] - path[i+1][1])

        cost = orginalCost - journey_storage_object[4][i]
        orginalCost = cost
        direction = getDirection(path[i-1], path[i], journey_storage_object[3][i], orginalDirection)
        orginalDirection = direction
        print("Cost", cost, "xdist, ydist", xdist, ydist, "Direction", direction)
        rows.append([direction, xdist, ydist, cost])
       # rows.append()
        if cost == 0:
            break
        # print(f"heading:{heading}, xdist:{xdist}, ydist:{ydist}, cost: {cost}\n")




    # TODO make method that backtracks and writes to csv per loop

def run(runTime):
        '''
        Main function to run the genetic algoritm

        Takes a time in seconds to run the algorithm for
        '''
        global rows
        t = time()
        i = 0
        while ((time() - t) < runTime):
            numCol = 100
            numRow = 100



            totalNodeCost = [0] * 7
            totalScore = [0] * 7

            board = generateRandomBoard.getBoard(numCol, numRow)  # Generating a random game board
            aStarData = pathPlanner.plan_path(board, 5)


            print("Starting Cost", aStarData[2])
            if( len(aStarData[0]) == len(aStarData[4])):
                write_to_csv(aStarData, board)


        rows = pd.DataFrame(rows, columns=fields)
        rows.to_csv('results2.csv', index=False)


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    # iterations = 6
    # numCol = 180
    # numRow = 180
    #
    #
    # totalNodeCost = [0] * 7
    # totalScore = [0] * 7
    #
    # board = generateRandomBoard.getBoard(numCol, numRow)  # Generating a random game board
    # aStarData = pathPlanner.plan_path(board, 5)
    #
    # write_to_csv(aStarData, board).run(5)

    run(600)

    # for x in range(iterations):
    #     board = generateRandomBoard.getBoard(numCol, numRow) # Generating a random game board
    #     aStarData = pathPlanner.plan_path(board, x+1)
    #
    #
    #     path = aStarData[0]
    #     totalNodeCost[x+1] += aStarData[1]
    #     totalScore[x+1] += aStarData[2]
    #     movesTaken = aStarData[3]
    #
    #     # print("error in iteration ", x)
    #     print("\nHeuristic %d" %(x+1))
    #     print("Path taken: ", path)
    #     print("Total Moves made: ", movesTaken)
    #
    #     #process = psutil.Process(os.getpid())
    #     #print(psutil.virtual_memory()[2])
    #     #print(process.memory_info().rss / (1024 * 1024), "MB")
    #     # Print our results
    #     print("Heuristic #", x+1, ": ", "Total Nodes Expanded: ", totalNodeCost[x+1], " Score: ",
    #           totalScore[x+1])