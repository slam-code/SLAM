# -*- coding: utf-8 -*-
# Write a function 'kalman_filter' that implements a multi-
# dimensional Kalman Filter for the example given

from math import *

class matrix:
    
    # implements basic operations of a matrix class
    
    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0
    
    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]
    
    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1
    
    def show(self):
        for i in range(self.dimx):
            print self.value[i]
        print ' '
    
    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to add"
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res
    
    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to subtract"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res
    
    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res
    
    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res
    
    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions
    
    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError, "Matrix not positive-definite"
                res.value[i][i] = sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
        return res
    
    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)
        
        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
        return res
    
    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res
    
    def __repr__(self):
        return repr(self.value)
'''
Python中这个_repr_函数，对应repr(object)这个函数，返回一个可以用来表示对象的可打印字符串：
尝试生成这样一个字符串，将其传给 eval()可重新生成同样的对象 ；
否则，生成用尖括号包住的字符串，包含类型名和额外的信息(比如地址) ；
一个类(class)可以通过 __repr__() 成员来控制repr()函数作用在其实例上时的行为。

str与repr区别：
1、python中str函数通常把对象转换成字符串，即生成对象的可读性好的字符串，一般在输出文本时使用，或者用于合成字符串。str的输出对用户比较友好适合print输出。
2、pyton中repr函数将一个对象转成类似源代码的字符串，只用于显示。repr的输出对python友好，适合eval函数得到原来的对象。

'''

########################################

# Implement the filter function below

def kalman_filter(x, P): #先测量,在移动,以下全是矩阵操作,没有单个的数
    for n in range(len(measurements)):
        pass
        # measurement update
        z=matrix([[measurements[n]]])  #第n次测量值,矩阵形式
        y= z-H*x

        s=H*P*(H.transpose())+R
        k=P*(H.transpose())*(s.inverse())
        x=x+k*y
        P=(I-k*H)*P

        # prediction ,预测step,

        x=F*x+u                       #相加
        P=F*P*(F.transpose())         #更新协方差

        print "x="
        x.show()

        print "P="
        P.show()


    return x,P

############################################
### use the code below to test your filter!
############################################


measurements = [1, 2, 3]
#measurements = [1, 2, 3,4,5,6,7]

x = matrix([[0.], [0.]]) # initial state (location and velocity)  2维高斯变量
P = matrix([[1000., 0.], [0., 1000.]]) # initial uncertainty      协方差矩阵,且2者不相关
u = matrix([[0.], [0.]]) # external motion                        外部运动矩阵,如加速度力
F = matrix([[1., 1.], [0, 1.]]) # next state function             状态转移矩阵, x'=x+delta_t*v;   v'=v
H = matrix([[1., 0.]]) # measurement function                     测量矩阵,只测量x,位置,而不测量v(现实无法测量)
R = matrix([[1.]]) # measurement uncertainty                      测量分布
I = matrix([[1., 0.], [0., 1.]]) # identity matrix                单位分布

print kalman_filter(x, P)
# output should be:
# x: [[3.9996664447958645], [0.9999998335552873]]
# P: [[2.3318904241194827, 0.9991676099921091], [0.9991676099921067, 0.49950058263974184]]


# print "---\n\n"

# a=matrix([[1.0,2.0],[3.,4.]])
# b=matrix([[5.0,6.],[7.,8.]])
# a.show()
# #b=a.transpose()
# b.show()
# (a*b).show() #矩阵相乘

#运行结果:
# x=
# [0.9990009990009988]  #第1次measurement和motion后,现在预测delta_t时间后位置是1,(因为此前速度是0)
# [0.0]                 #     速度是0
 
# P=
# [1000.9990009990012, 1000.0] # 方差大,不确定性大
# [1000.0, 1000.0]
 
# x=
# [2.998002993017953]  #第2次measurement和motion后,现在预测delta_t时间后位置是3,(因为此时速度是1,此处大幅度已矫正)
# [0.9990019950129659] # 速度是1
 
# P=
# [4.990024935169789, 2.9930179531228447]
# [2.9930179531228305, 1.9950129660888933]
 
# x=
# [3.9996664447958645] #第3次measurement和motion,现在预测delta_t时间后位置是4,(此时协方差矩阵很小,对位置的估计准确性很高.
# [0.9999998335552873] #速度是1
 
# P=
# [2.3318904241194827, 0.9991676099921091]
# [0.9991676099921067, 0.49950058263974184]
 
