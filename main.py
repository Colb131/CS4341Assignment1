import pathPlanner
import generateRandomBoard
import csv
import numpy as np

aStarData = [None] * 4


def reader():
    file = input("Please enter file name: ")

    data = np.loadtxt(file, delimiter="\t", dtype=str)
    data = np.transpose(data)
    return data


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    iterations = 1
    numCol = 10
    numRow = 5

    totalNodeCost = 0
    totalScore = 0

    print("Running ", iterations, " iterations of board size (", numCol, ",", numRow, ")")

    for x in range(iterations):
        board = reader()  # Gives us a ndarray
        print(board.dtype)

        if iterations == 1:
            print(board)
        newBoard = []
        for arr in board:
            newRow = []
            for val in arr:
                newRow.append(ord(val) - 48)
            newBoard.append(newRow)
        board = np.array(newBoard)
        print(board)

        heuristicOption = int(input("Please enter the desired heuristic: "))

        aStarData = pathPlanner.plan_path(board, heuristicOption)

        path = aStarData[0]
        totalNodeCost += aStarData[1]
        totalScore += aStarData[2]

        if iterations == 1:
            print(len(path))

        # for i in range(1, 7):
        #     aStarData = pathPlanner.plan_path(board, i)
        #
        #     path = aStarData[0]
        #     totalNodeCost[i - 1] += aStarData[1]
        #     totalScore[i - 1] += aStarData[2]
        #
        #     if iterations == 1:
        #         print(len(path))
        # process = psutil.Process(os.getpid())
        # print(psutil.virtual_memory()[2])
        # print(process.memory_info().rss / (1024 * 1024), "MB")
    print("Heuristic #", heuristicOption, ": ", "Average Total Nodes Expanded: ", totalNodeCost / iterations,
            " Average Score: ",
               totalScore / iterations)
    # for i in range(1, 7):  # Print our results
    #     print("Heuristic #", i, ": ", "Average Total Nodes Expanded: ", totalNodeCost[i - 1] / iterations,
    #           " Average Score: ",
    #           totalScore[i - 1] / iterations)
