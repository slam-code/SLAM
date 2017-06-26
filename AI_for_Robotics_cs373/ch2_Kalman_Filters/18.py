# -*- coding: utf-8 -*-

def f(mu, sigma2, x):
    return 1/sqrt(2.*pi*sigma2) * exp(-.5*(x-mu)**2 / sigma2)

#高斯分布+后的分布
# Write a program that will predict your new mean
# and variance given the mean and variance of your 
# prior belief and the mean and variance of your 
# motion. 

def update(mean1, var1, mean2, var2):
    new_mean = (var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1/(1/var1 + 1/var2)
    return [new_mean, new_var]

def predict(mean1, var1, mean2, var2): #运动本身造成了方差增大,均值增加
    new_mean =mean1+mean2
    new_var =var1+var2
    return [new_mean, new_var]

print predict(10., 4., 12., 4.)