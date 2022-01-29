import pathPlanner
import generateRandomBoard
import generateRandomBoard as Board_Gen

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    board = generateRandomBoard.getBoard()
    print(board)
    path = pathPlanner.plan_path(board)
    print('Hi')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
