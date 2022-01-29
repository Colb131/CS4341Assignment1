import pathPlanner
import generateRandomBoard
import generateRandomBoard as Board_Gen

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    board = generateRandomBoard.getBoard()
    print(board)
    gameboard = [[-1,2,3,4],
                       [5,6,7,8],
                       [1,2,3,4],
                       [5,6,7,-2]]
    #print(gameboard)
    path = pathPlanner.plan_path(board)
    print(path)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
