# -*- coding: utf-8 -*-
# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space
import copy
grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
grid = [[0, 1],
        [0, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1] #右下角
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost):
    # ----------------------------------------
    # insert code here
    # ----------------------------------------
    closed=copy.deepcopy(grid)
    #print init[0],init[1],closed
    #xx=init[0]
    #yy=init[1]
    #print type(xx)
    #closed[init[0],init[1]]=1
    #closed[xx][yy]=1
    # closed=[]
    # for i in range(len(grid)):
    #     #for j in range(len[grid[i]]):
    #         closed.append(grid[i])

    closed[init[0]][init[1]]=1

    openlist=[[0,init[0],init[1]]]
    #print openlist
    paths=[]
    while len(openlist)>0:
        for i in xrange(4):
            #print openlist[0][1],openlist[0][2]
            nowxy=[openlist[0][1],openlist[0][2]]
            dxy=delta[i]
            temp=[nowxy[0]+dxy[0],nowxy[1]+dxy[1]]
            #print temp
            if temp[0]<len(closed) and  temp[0]>=0 and temp[1]<len(closed[0]) \
            and temp[1]>=0  and closed[temp[0]][temp[1]]==0:
                closed[temp[0]][temp[1]]=1
                openlist.append([openlist[0][0]+1,temp[0],temp[1]])
            
        if openlist[0][1]==goal[0] and openlist[0][2]==goal[1]:
            paths.append(openlist[0])
        del openlist[0]


    paths.sort()
    if len(paths)==0:
        return "fail"
    return paths[0]
print search(grid,init,goal,cost)
#print init