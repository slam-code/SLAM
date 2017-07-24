def sense(colors,measurements,sensor_right,p):
    
    for m in range(len(measurements)):
        sump=0.0
        for i in range(len(p)):
            for j in range(len(p[i])):
                Hit=(measurements[m]==colors[i][j])
                s=p[i][j]*Hit*sensor_right+p[i][j]*(1-Hit)*(1-sensor_right)
                p[i][j]=s
                sump+=s;
        for i in range(len(colors)):
                for j in range(len(colors[i])):
                    p[i][j]/=sump
    return p

def show(p):
    rows = ['[' + ','.join(map(lambda x: '{0:.5f}'.format(x),r)) + ']' for r in p]
    print '[' + ',\n '.join(rows) + ']\n'


def motion(colors,motions,p,p_move):
    for i in range(len(motions)):

            q = [[0.0 for row in range(len(colors[0]))] for col in range(len(colors))]
            step=motions[i]
            dy=step[0]
            dx=step[1]
            sump=0.0 
            for i in range(len(p)):
                for j in range(len(p[i])):
                    pmovey=(i-dy)%len(p)
                    pmovex=(j-dx)%len(p[i])
                    #print pmovex,pmovey,p[pmovex][pmovey]
                    q[i][j]=p[pmovey][pmovex]*p_move+p[i][j]*(1-p_move)
                    sump+=q[i][j] 
            for i in range(len(colors)):
                for j in range(len(colors[i])):
                    q[i][j]/=sump
            p=q        
            show(p)
    return p

colors = [['R','R','R'],
          ['R','G','R'],
          ['R','R','R']]

measurements=['G']
motions = [[0,0],[0,1],[1,0],[0,1]]
#motions = [[0,0]]
sensor_right=1
p_move = 1

pinit = 1.0 / float(len(colors)) / float(len(colors[0]))
p = [[pinit for row in range(len(colors[0]))] for col in range(len(colors))]

p=sense(colors,measurements,sensor_right,p)
show(p)
print "<-sense\n"

#l=[10,11,12]
#print 0%3,-1%3,-2%3,-3%3
#0,2,1,0
#print l[0%3],l[-1%3],l[-2%3],l[-3%3]


p=motion(colors,motions,p,p_move)
# show(p)
# print "<- motion"