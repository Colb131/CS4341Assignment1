import PathPlanner as Path_Planner
import generateRandomBoard as Board_Gen

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    boardGen = Board_Gen()
    pPlaner = Path_Planner()
    board = boardGen.gen_board()
    path = pPlaner.plan_path(board)
    print('Hi')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
