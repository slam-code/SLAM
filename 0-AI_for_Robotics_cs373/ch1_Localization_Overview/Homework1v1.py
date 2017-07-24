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



colors = [['R','R','R'],
          ['R','G','G'],
          ['R','R','R']]
#measurements = ['G','G','G','G','G']
measurements=['G']
#motions = [[0,0],[0,1],[1,0],[1,0],[0,1]]
sensor_right=1.0
p_move = 0.8

pinit = 1.0 / float(len(colors)) / float(len(colors[0]))
p = [[pinit for row in range(len(colors[0]))] for col in range(len(colors))]

# p=sense(colors,measurements,sensor_right,p)
# show(p)
# [[0.00000,0.00000,0.00000],
#  [0.00000,0.50000,0.50000],
#  [0.00000,0.00000,0.00000]]

p=sense(colors,measurements,0.8,p)
show(p)

# [[0.06667,0.06667,0.06667],
#  [0.06667,0.26667,0.26667],
#  [0.06667,0.06667,0.06667]]
