import pathPlanner
import generateRandomBoard
import csv
import numpy as np

aStarData = [None] * 4


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




# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    iterations = 1
    #numCol = 10
    #numRow = 10

    totalNodeCost = [0] * 7
    totalScore = [0] * 7

    for x in range(iterations):
        board = reader() # Gives us a ndarray

        # np.array([s[0].astype(int) for s in board])

       #board = generateRandomBoard.getBoard(numCol, numRow) # Generating a random game board
        if iterations == 1:




            aStarData = pathPlanner.plan_path(board, heuristicOption)


            path = aStarData[0]
            totalNodeCost[heuristicOption] += aStarData[1]
            totalScore[heuristicOption] += aStarData[2]
            movesTaken = aStarData[3]

            # print("error in iteration ", x)
            print("")
            if iterations == 1:

                print("Path taken: ", path)
                print(movesTaken)

            #process = psutil.Process(os.getpid())
            #print(psutil.virtual_memory()[2])
            #print(process.memory_info().rss / (1024 * 1024), "MB")
        # Print our results
        print("Heuristic #", heuristicOption, ": ", "Total Nodes Expanded: ", totalNodeCost[heuristicOption], " Score: ",
              totalScore[heuristicOption])