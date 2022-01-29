#!/usr/bin/env python

import numpy as np
from numpy import *
arrayCols = 12
arrayRows = 12

def placeS(arrayCols, arrayRows, board):
    startCol = np.random.randint(0, arrayCols - 1)
    startRow = np.random.randint(0, arrayRows - 1)
    if board[startCol][startRow] == -1:
        placeS(arrayCols, arrayRows, board)
    else:
        board[startCol][startRow] = -2


def getBoard():

    board = np.random.randint(10, size=(arrayRows, arrayCols))

    goalCol = np.random.randint(0, arrayCols - 1)
    goalRow = np.random.randint(0, arrayRows - 1)

    board[goalCol][goalRow] = -1

    placeS(arrayCols, arrayRows, board)

    return board

