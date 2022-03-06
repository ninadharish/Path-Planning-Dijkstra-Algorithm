## ENPM661 Project 2
## Ninad Harishchandrakar - 118150819 - ninadh@umd.edu

import numpy as np
import cv2



def userInput():

    initial_x = input("Enter x xoordinate of starting point: ")
    initial_y = input("Enter y xoordinate of starting point: ")

    goal_x = input("Enter x xoordinate of goal point: ")
    goal_y = input("Enter y xoordinate of goal point: ")

    initial = [initial_x, initial_y]
    goal = [goal_x, goal_y]

    return initial, goal
        


def visualize():

    vis = np.zeros((400, 250, 3), np.uint8)

    for x in range(400):
        for y in range(250):

            if (((x - 300)**2 + (y - 185)**2 < 45**2)) or ((1.2318840579710144 * x) + y > 221.41444986323697) and ((-0.31645569620253167 * x) + y < 178.85198290781432) and (((-0.8571428571428571 * x) + y > 104.84318253050509) or ((3.2 * x) + y < 452.7630546142402)) or (((0.5773502691896258 * x) + y < 261.65807537309524) and (x < 240) and ((-0.5773502691896256 * x) + y > -61.65807537309518) and ((0.5773502691896256 * x) + y > 169.28203230275506) and (x > 160) and ((-0.5773502691896258 * x) + y < 30.717967697244905)) or (x <= 5) or (x >= 395) or (y <= 5) or (y >= 245):
                
                vis[x][y] = (0, 255, 255)

            if ((x - 300)**2 + (y - 185)**2 < 40**2) or (((0.5773502691896258 * x) + y < 255.88457268119896) and (x < 235) and ((-0.5773502691896256 * x) + y > -55.88457268119893) and ((0.5773502691896256 * x) + y > 175.05553499465134) and (x > 165) and ((-0.5773502691896258 * x) + y < 24.944465005348647) or ((1.2318840579710144 * x) + y > 229.3478260869565) and ((-0.31645569620253167 * x) + y < 173.60759493670886) and (((-0.8571428571428571 * x) + y > 111.42857142857143) or ((3.2 * x) + y < 436.0))):

                vis[x][y] = (0, 0, 255)

    return vis



def free_space():

    space = np.full((400, 250), np.inf)

    for x in range(400):
        for y in range(250):

            if (((x - 300)**2 + (y - 185)**2 < 45**2)) or ((1.2318840579710144 * x) + y > 221.41444986323697) and ((-0.31645569620253167 * x) + y < 178.85198290781432) and (((-0.8571428571428571 * x) + y > 104.84318253050509) or ((3.2 * x) + y < 452.7630546142402)) or (((0.5773502691896258 * x) + y < 261.65807537309524) and (x < 240) and ((-0.5773502691896256 * x) + y > -61.65807537309518) and ((0.5773502691896256 * x) + y > 169.28203230275506) and (x > 160) and ((-0.5773502691896258 * x) + y < 30.717967697244905)) or (x <= 5) or (x >= 395) or (y <= 5) or (y >= 245):
                
                space[x][y] = -1

    return space



def isValid(point, space):

    if (not (0 < point[0] < 400)) or (not (0 < point[1] < 250)) or (space[point[0]][point[1]] == -1):
        return False
    else:
        return True



def create_parent_map():

    parent = [[[0 for col in range(2)] for col in range(250)] for row in range(400)]

    return parent



def moveAction(node, open_list, closed_list, space, parent):

    act_set = [[1,0], [-1,0], [0,1], [0,-1], [1,1], [-1,1], [1,-1], [-1,-1]]
    L = [1, 1, 1, 1, 1.4, 1.4, 1.4, 1.4]
    new_node = []

    for i in range(8):

        new_node.append([(node[0] + act_set[i][0]), (node[1] + act_set[i][1])])

        if (not (new_node[i] in closed_list)) and (isValid(new_node[i], space)):
            
            if (not (tuple(new_node[i]) in open_list)) or (space[new_node[i][0]][new_node[i][1]] == np.inf):

                parent[new_node[i][0]][new_node[i][1]] = node
                space[new_node[i][0]][new_node[i][1]] = space[node[0]][node[1]] + L[i]
                open_list[(new_node[i][0], new_node[i][1])] = space[new_node[i][0]][new_node[i][1]]

            else:

                if space[new_node[i][0]][new_node[i][1]] > space[node[0]][node[1]] + L[i]:
                    parent[new_node[i][0]][new_node[i][1]] = node
                    space[new_node[i][0]][new_node[i][1]] = space[node[0]][node[1]] + L[i]
                    open_list[(new_node[i][0], new_node[i][1])] = space[new_node[i][0]][new_node[i][1]]



def backtrack(parent, goal):

    node = goal
    path = []
    path.append(goal)

    while tuple(parent[node[0]][node[1]]) != (-1, -1):
        node = parent[node[0]][node[1]]
        path.append(node)

    return path[::-1]



def dijkstra(initial, goal):

    space = free_space()
    vis = visualize()

    if (not isValid(initial, space)) or (not isValid(goal, space)):
        print("One of the two nodes is not valid. Please try again.")

    else:
        open_list = {}
        closed_list = []
        parent = create_parent_map()
        goal_reached = False

        space[initial[0]][initial[1]] = 0
        parent[initial[0]][initial[1]] = [-1, -1]

        open_list[tuple([initial[0], initial[1]])] = space[initial[0]][initial[1]]

        current_node = initial

        while (open_list) and (not goal_reached):

            closed_list.append(current_node)

            if current_node[0] == goal[0] and current_node[1] == goal[1]:
                goal_reached = True
                opt_path = backtrack(parent, goal)
                for item in opt_path:
                    vis[item[0]][item[1]] = (255, 0, 0)
                print("Optimal Path by Dijkstra Algorithm is depicted in Blue")
                vis = cv2.rotate(vis, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv2.imshow('Dijkstra', vis)
                key = cv2.waitKey(0)
                if key == 27:
                    cv2.destroyAllWindows()
                
                break

            else:
                
                moveAction(current_node, open_list, closed_list, space, parent)
            
            vis[current_node[0]][current_node[1]] = (0, 255, 0)
            vis = cv2.rotate(vis, cv2.ROTATE_90_COUNTERCLOCKWISE)

            cv2.imshow('Dijkstra', vis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return 0

            vis = cv2.rotate(vis, cv2.ROTATE_90_CLOCKWISE)

            open_list.pop(tuple(current_node), None)

            current_node = list(min(open_list, key=open_list.get))



def main():

    initial, goal = userInput()

    dijkstra(initial, goal)


main()








        