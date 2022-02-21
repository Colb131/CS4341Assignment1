import numpy as np
from sklearn.linear_model import LinearRegression
from _csv import reader



"""
code taken from https://realpython.com/linear-regression-in-python/
"""


# TODO read in file from csv to arrays

# TODO reformat the arrays so the scores are in 1 array and the x_dist and y_dist are in pairs in another array

# TODO find out how to get the multiple factors of linear regression to work

# TODO export values to the learned_values file
def perform_regression():
    read_obj = open("results2.csv", 'r')
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
        print(independents)
        print(dependents)

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
    f = open("results2.csv", 'w')
    f.write("heading, x-dist, y-dist, cost-to-goal\n")
    f.close()
