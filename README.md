# CS4341Assignment1
Group 25
Keval Ashara
Colby Frechette
Amy Orozco

Please see the below google doc for our writeup.

https://docs.google.com/document/d/10kfMUZYGlbUR6iL2B0Im1S2orYvjYmWn-wUBEQeErRA/edit?usp=sharing


## Notes to run

Simply put the desired text file into the CS4341Assignment1 folder 

In terminal, run python3 and execute "main.py" to start the program.

Enter the file's name (without quotes) ex: "input.txt"

Enter the selected heuristic (1-7)


## Above is verbatim the previous groups instructions adding in heuristic 7


Our added functionality is :

* randomgen.py can be run and train the algorithm for a (command line input) number of seconds outputting the linear regression values, and the average nodes expanded and score for heuristic 5, 6, and 7
* main works as originally did, running with the board file and heuristic number from command line, but also stores the values to the results.csv and performs linear regression with results.csv and outputs those values
* linear_regression can be run with no command line and it erases the learned_values and the results files after performing regression analysis and outputting the values