import pathPlanner
import generateRandomBoard
import csv
import numpy as np

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

    iterations = 100
    numCol = 100
    numRow = 100

    print("Running ", iterations, " iterations of board size (", numCol, ",", numRow,")")
    for i in range(1,7):
        totalNodeCost = 0
        totalScore = 0
        for x in range(iterations):
            board = generateRandomBoard.getBoard(numCol, numRow)

            #board = reader()

            #TODO: Add a command line for this, 1-6
            heuristicOption = i

            aStarData = pathPlanner.plan_path(board, heuristicOption)

            path = aStarData[0]
            totalNodeCost += aStarData[1]
            totalScore += aStarData[2]
            #print(path)
        print("Heuristic #", i, ": ", "Average Node Cost: ", totalNodeCost/iterations, " Average Score: ", totalScore/iterations)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
