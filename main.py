import pathPlanner
import generateRandomBoard
import csv
import numpy as np
import os, psutil

aStarData = [None] * 3

def reader():
    with open('input.txt', 'r') as f:
        data = f.readlines()
    result = []
    for raw_line in data:
        split_line = raw_line.strip().split("\t")  # ["1", "0" ... ]
        result.append(split_line)

        #data = csv.reader(f, dialect='excel', delimiter='\t')
    #x = list(data)
    #result = np.array(x)

    result = np.asmatrix(result)
    result = np.where(result == 'G', -1, result)
    result = np.where(result == 'S', -2, result)
    result.astype(int)
    #print(result)
    return result




# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    iterations = 10
    numCol = 10
    numRow = 10

    totalNodeCost = [0] * 6
    totalScore = [0] * 6

    print("Running ", iterations, " iterations of board size (", numCol, ",", numRow,")")

    for x in range(iterations):
        board = generateRandomBoard.getBoard(numCol, numRow)

        if iterations == 1:
            print(board)
        #print(x)
        for i in range(1,7):
            # board = reader()

            # TODO: Add a command line for this, 1-6
            heuristicOption = i

            aStarData = pathPlanner.plan_path(board, heuristicOption)


            path = aStarData[0]
            totalNodeCost[i-1] += aStarData[1]
            totalScore[i-1] += aStarData[2]

            # print("error in iteration ", x)

            if iterations == 1:
                print(path)
        process = psutil.Process(os.getpid())
        print(process.memory_info().rss)

    for i in range(1, 7): # Print our results
        print("Heuristic #", i, ": ", "Average Node Cost: ", totalNodeCost[i-1] / iterations, " Average Score: ",
              totalScore[i-1] / iterations)