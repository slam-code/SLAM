#Program a function that returns a new distribution 
#q, shifted to the right by U units. If U=0, q should 
#be the same as p.

p=[0, 1, 0, 0, 0]
world=['green', 'red', 'red', 'green', 'green']
measurements = ['red', 'green']
pHit = 0.6
pMiss = 0.2

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
    #
    #ADD CODE HERE
    #
    U = U % len(p)
    q = p[-U:] + p[:-U]
    return q
    #或者:
    for x in xrange(len(p)):
        q.append((i-U)%len(p))
    return q
        
        
        
print move(p, 1)
#[0, 0, 1, 0, 0]
