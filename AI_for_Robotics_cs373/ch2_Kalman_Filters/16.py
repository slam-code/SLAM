# -*- coding: utf-8 -*-
# Write a program to update your mean and variance
# when given the mean and variance of your belief
# and the mean and variance of your measurement.
# This program will update the parameters of your
# belief function.

from math import *

def f(mu, sigma2, x):
    return 1/sqrt(2.*pi*sigma2) * exp(-.5*(x-mu)**2 / sigma2)

#高斯分布相乘后的分布,
def update(mean1, var1, mean2, var2):
    new_mean = 1.0/(var1+var2)*(var2*mean1+var1*mean2)
    new_var =1.0/(1.0/var1+1.0/var2)
    return [new_mean, new_var]

print update(10.,8.,13., 2.)