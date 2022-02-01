#!/usr/bin/env python

import numpy as np
from numpy import *

def placeS(arrayCols, arrayRows, board):
    startCol = np.random.randint(0, arrayCols - 1)
    startRow = np.random.randint(0, arrayRows - 1)
    if board[startCol][startRow] == ord('G')-48:
        placeS(arrayCols, arrayRows, board)
    else:
        board[startCol][startRow] = ord('S')-48


def getBoard(cols, rows):
    arrayCols = cols
    arrayRows = rows

    board = np.random.randint(9, size=(arrayCols, arrayRows))+1

    goalCol = np.random.randint(0, arrayCols - 1)
    goalRow = np.random.randint(0, arrayRows - 1)

    board[goalCol][goalRow] = ord('G')-48

    placeS(arrayCols, arrayRows, board)

    nparr= np.array(board)
    nparr = np.where(board == -1, ord('G')-48, board)
    # print(nparr)
    nparr = np.where(board == -2, ord('S')-48, nparr)
    print(nparr)

    return board

