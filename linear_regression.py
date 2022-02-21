from _csv import reader

import numpy as np
from sklearn.linear_model import LinearRegression

"""
code taken from https://realpython.com/linear-regression-in-python/
"""


def perform_regression():
    read_obj = open("results.csv", 'r')
    csv_reader = reader(read_obj)
    header = next(csv_reader)
    if header is not None:
        dependents = []
        independents = []
        for row in csv_reader:
            dependents.append(float(row[-1]))
            row.pop()
            for i in row:
                independents.append(float(i))
        independents = np.array(independents).reshape(-1, 3)
       # print(independents)
       # print(dependents)

        model = LinearRegression().fit(independents, dependents)
        print(f"R^2:{model.score(independents, dependents)}")
        print(f"intercept:{model.intercept_}")
        print(f"slopes:{model.coef_}")
        clear_learned = open("learned_values.txt", 'w')
        clear_learned.close()
        store_values = open("learned_values.txt", 'a')
        for i in model.coef_:
            store_values.write(f"{i}\n")
        store_values.write(str(model.intercept_))


if __name__ == '__main__':
    perform_regression()
    # resets the csv to only have the header
    f = open("results.csv", 'w')
    f.write("heading, x-dist, y-dist, cost-to-goal\n")
    f.close()
