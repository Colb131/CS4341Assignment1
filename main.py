import pathPlanner
import generateRandomBoard
import csv
import numpy as np

matrix = []

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
    print(result)
    return result




# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    board = generateRandomBoard.getBoard()

    #board = reader()

    #TODO: Add a command line for this, 1-6
    heuristicOption = 6

    path = pathPlanner.plan_path(board, heuristicOption)
    print(path)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
