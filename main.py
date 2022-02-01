import pathPlanner
import generateRandomBoard
import csv
import numpy as np

aStarData = [None] * 4

def reader():
    file = input("Please enter file name: ")
    # with open(file, 'r') as f:
    #     reader = csv.reader(f, dialect='excel', delimiter='\t')
    #     x = list(reader)
    #     result = np.array(x)

    data = np.loadtxt(file, delimiter="\t", dtype=str)
    data = np.transpose(data)
    print(data.size)
    #result = np.asmatrix(result)
    # matrix = np.asmatrix(result)
    # result = np.where(result == 'G', -1, result)
    # result = np.where(result == 'S', -2, result)
    # result.astype(int)

    print(data.dtype)
    # heuristicOption = int(input("Please enter desired heuristic: "))

    newBoard = []
    for arr in data:
        newRow = []
        for val in arr:
            newRow.append(ord(val) - 48)
        newBoard.append(newRow)
    # print(newBoard)
    board = np.array(newBoard)
    #print(board)
    #print(type(board))

    return board




# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    iterations = 1
    numCol = 300
    numRow = 300

    totalNodeCost = [0] * 6
    totalScore = [0] * 6

    print("Running ", iterations, " iterations of board size (", numCol, ",", numRow,")")

    for x in range(iterations):
        #board = reader() # Gives us a ndarray

        # np.array([s[0].astype(int) for s in board])

        board = generateRandomBoard.getBoard(numCol, numRow) # Generating a random game board
        if iterations == 1:
            print(board)
            print(type(board))

        for i in range(1,7):


            aStarData = pathPlanner.plan_path(board, i)


            path = aStarData[0]
            totalNodeCost[i-1] += aStarData[1]
            totalScore[i-1] += aStarData[2]

            # print("error in iteration ", x)

            if iterations == 1:
                print(len(path))
        #process = psutil.Process(os.getpid())
        #print(psutil.virtual_memory()[2])
        #print(process.memory_info().rss / (1024 * 1024), "MB")
    for i in range(1, 7): # Print our results
        print("Heuristic #", i, ": ", "Average Total Nodes Expanded: ", totalNodeCost[i-1] / iterations, " Average Score: ",
              totalScore[i-1] / iterations)