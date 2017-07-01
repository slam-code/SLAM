# -*- coding: utf-8 -*-
# ----------
# User Instructions:
# 
# Create a function compute_value which returns
# a grid of values. The value of a cell is the minimum
# number of moves required to get from the cell to the goal. 
#
# If a cell is a wall or it is impossible to reach the goal from a cell,
# assign that cell a value of 99.
# ----------
import copy
grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1 # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']

def compute_value(grid,goal,cost):
    # ----------------------------------------
    # insert code below
    # ----------------------------------------
    
    # make sure your function returns a grid of values as 
    # demonstrated in the previous video.
    value=copy.deepcopy(grid)
    for i in range(len(value)):
        for j in range(len(value[i])):
            #if value[i][j]==1:
                value[i][j]=99

    visited=copy.deepcopy(grid)
    #visited[goal[0]][visited[goal[1]]]=1
    x=goal[0]
    y=goal[1]
    openlist=[[x,y]]
    value[x][y]=0

    while len(openlist)>0:
        
        nowxy=openlist[0]
        for i in range(len(delta)):
             #print nowxy[0],delta[i][0]
             dx=nowxy[0]+delta[i][0]
             dy=nowxy[1]+delta[i][1]
             if dx>=0 and dx<len(grid) and dy>=0 and dy<len(grid[dx]) and grid[dx][dy]==0:
                       
                        if value[dx][dy]> value[nowxy[0]][nowxy[1]]+1:
                            value[dx][dy]=value[nowxy[0]][nowxy[1]]+1
                            openlist.append([dx,dy])

        visited[nowxy[0]][nowxy[1]]=1
        del openlist[0]
            

    return value 

res= compute_value(grid,goal,cost)

for i in range(len(res)):
    print res[i]