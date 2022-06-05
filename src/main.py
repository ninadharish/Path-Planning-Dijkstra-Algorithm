import numpy as np
from dijkstra import *


if __name__ == "__main__":

    initial, goal = userInput()
    dijkstra(initial, goal)