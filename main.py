import pathPlanner
import generateRandomBoard
import csv
import numpy as np

matrix = []

def reader():
    with open('input.txt', 'r') as f:
        reader = csv.reader(f, dialect='excel', delimiter='\t')
    x = list(reader)
    result = np.array(x)

    print(result)

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    board = generateRandomBoard.getBoard()

    #TODO: Add a command line for this, 1-6
    heuristicOption = 1

    path = pathPlanner.plan_path(board, heuristicOption)
    print(path)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
