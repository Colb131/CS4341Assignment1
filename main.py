import pathPlanner
import generateRandomBoard
import csv
import numpy as np
import sys

aStarData = [None] * 4


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


def write_to_csv(journey_storage_object, input_filename):
    board_copy = reader(input_filename)
    journey_storage_object[3].reverse()
    journey_storage_object[0].reverse()
    backtrack_array = [journey_storage_object[2]]
    goal_point = journey_storage_object[0][0]
    # TODO write x dist = 0 y dist = 0 cost = final_cost
    for i in range(journey_storage_object[3]):

        pass
    # TODO make method that backtracks and writes to csv per loop


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    iterations = 1
    # numCol = 10
    # numRow = 10

    totalNodeCost = [0] * 7
    totalScore = [0] * 7

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
            write_to_csv(aStarData, filename)
        # Print our results
        print("Heuristic #", heuristic, ": ", "Total Nodes Expanded: ", totalNodeCost[heuristic],
              " Score: ",
              totalScore[heuristic])
