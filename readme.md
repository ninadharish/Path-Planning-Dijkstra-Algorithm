![alt text](/output/output.gif)

# Path Planning with the Dijkstra Algorithm

## Description

The [Dijkstra Algotihm](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm) is used widely to traverse a graph between nodes and finding the optimal path. It is also extensively used for Robot Path Planning. For a given Obstacle Space, this program applies the Dijkstra Algorithm for Path Planning in order to find the optimal path for the robot to travel from a given initial potiion to a given goal position taking into consideration all the obstacles as well as a clearance space around them.

## Approach

Algorithm:

![alt text](/output/flo.png)

## Output

The 2D Obstacle Space where the Obstacled are shown in red and a clearance space around the obstacles is shown in yellow.

![alt text](/output/output1.png)

The path from the initial point (6,6) to the goal point (130,100) is found using the Dijkstra Algorithm. The nodes explored are depicted in green. The optimal path is shown in blue.

![alt text](/output/output2.png)


The Video Output can be found [here](https://drive.google.com/file/d/1oppuylvXl61TDRY4Bmon5KPglXFWiiMg/view?usp=sharing).


## Getting Started

### Dependencies

<p align="left"> 
<a href="https://www.python.org" target="_blank" rel="noreferrer"> <img src="https://raw.githubusercontent.com/devicons/devicon/master/icons/python/python-original.svg" alt="python" width="40" height="40"/>&ensp; </a>
<a href="https://numpy.org/" target="_blank" rel="noreferrer"> <img src="https://www.codebykelvin.com/learning/python/data-science/numpy-series/cover-numpy.png" alt="numpy" width="40" height="40"/>&ensp; </a>
<a href="https://opencv.org/" target="_blank" rel="noreferrer"> <img src="https://avatars.githubusercontent.com/u/5009934?v=4&s=400" alt="opencv" width="40" height="40"/>&ensp; </a>

* [Python 3](https://www.python.org/)
* [NumPy](https://numpy.org/)
* [OpenCV](https://opencv.org/)


### Executing program

* Clone the repository into any folder of your choice.
```
git clone https://github.com/ninadharish/Path-Planning-Dijkstra-Algorithm.git
```

* Open the repository and navigate to the `src` folder.
```
cd Path-Planning-Dijkstra-Algorithm/src
```

* Run the program.
```
python main.py
```

* Follow the on screen instructions to input the initial and goal points in order to plan the path of the robot using the Dijkstra algorithm.


## Authors

👤 **Ninad Harishchandrakar**

* [GitHub](https://github.com/ninadharish)
* [Email](ninad.harish@gmail.com)
* [LinkedIn](https://linkedin.com/in/ninadharish)
