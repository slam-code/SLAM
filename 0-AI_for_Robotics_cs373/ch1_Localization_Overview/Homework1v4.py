def sense(colors,measurement,sensor_right,p):
        sump=0.0
        for i in range(len(p)):
            for j in range(len(p[i])):
                Hit=(measurement==colors[i][j])
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


def motion(colors,step,p,p_move):
            q = [[0.0 for row in range(len(colors[0]))] for col in range(len(colors))]
            dy=step[0]
            dx=step[1]
            sump=0.0 
            for i in range(len(p)):
                for j in range(len(p[i])):
                    pmovey=(i-dy)%len(p)
                    pmovex=(j-dx)%len(p[i])
                    q[i][j]=p[pmovey][pmovex]*p_move+p[i][j]*(1-p_move)
                    sump+=q[i][j] 
            for i in range(len(colors)):
                for j in range(len(colors[i])):
                    q[i][j]/=sump
            p=q        
            
            return p

colors = [['R','G','G','R','R'],
          ['R','R','G','R','R'],
          ['R','R','G','G','R'],
          ['R','R','R','R','R']]
measurements = ['G','G','G','G','G']
motions = [[0,0],[0,1],[1,0],[1,0],[0,1]]

sensor_right=0.7
p_move = 0.8

pinit = 1.0 / float(len(colors)) / float(len(colors[0]))
p = [[pinit for row in range(len(colors[0]))] for col in range(len(colors))]

for i in range(len(motions)):
   
    p=motion(colors,motions[i],p,p_move)
    print "motion i=",i
    show(p)

    p=sense(colors,measurements[i],sensor_right,p)
    print "sense i=",i
    show(p)


show(p)

#mint:05:02