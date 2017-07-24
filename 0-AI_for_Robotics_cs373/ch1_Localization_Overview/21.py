# -*- coding: utf-8 -*-
#Modify the move function to accommodate the added 
#probabilities of overshooting or undershooting 
#the intended destination.

p=[0, 1, 0, 0, 0]
world=['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
pHit = 0.6
pMiss = 0.2
pExact = 0.8
pOvershoot = 0.1
pUndershoot = 0.1

def sense(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)
    for i in range(len(q)):
        q[i] = q[i] / s
    return q

def move(p, U):
    q = []
     
    for i in range(len(p)):
        
        s=pExact*p[(i-U)%len(p)]
        s=s+pOvershoot*p[(i-U-1)%len(p)]
        s=s+pUndershoot*p[(i-U+1)%len(p)]
        q.append(s)

    return q
    

print move(p, 1)
#[0.0, 0.1, 0.8, 0.1, 0.0]


for i in range(10000):
    p=move(p,1) #度量更新

print p

#[0.20000000000000365, 0.20000000000000373, 0.20000000000000365, 0.2000000000000035, 0.2000000000000035]
#why? 机器人在不断的移动过程中,对自身的位置定位概率是不断的下降的,最后以至于无法确定自己位置.(因此需要测量装置,以便感知环境)