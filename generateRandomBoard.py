import numpy as np
from numpy import *

a = array([[4, "G", 4, 6],
           [2, 9, 9, 6],
           [1, 4, "S", 3]])

arrayCols = 4
arrayRows = 3

array = np.random.randint(10, size=(arrayRows, arrayCols))

goalCol = np.random.randint(0, arrayCols - 1)
goalRow = np.random.randint(0, arrayRows - 1)

array[goalCol][goalRow] = -1


def placeS():
    startCol = np.random.randint(0, arrayCols - 1)
    startRow = np.random.randint(0, arrayRows - 1)
    if array[startCol][startRow] == -1:
        placeS()
    else:
        array[startCol][startRow] = -2

placeS()

print(array)
