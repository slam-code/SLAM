# -*- coding: utf-8 -*-
#For this problem, you aren't writing any code.
#Instead, please just change the last argument 
#in f() to maximize the output.

from math import *

def f(mu, sigma2, x):
    return 1/sqrt(2.*pi*sigma2) * exp(-.5*(x-mu)**2 / sigma2)

print f(10.,4.,10.) #Change the 8. to something else!

#这里的卷积即全概公式,相加