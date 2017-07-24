# -*- coding: utf-8 -*-
# Write a program that will iteratively update and
# predict based on the location measurements 
# and inferred motions shown below. 

def update(mean1, var1, mean2, var2):
    new_mean = float(var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1./(1./var1 + 1./var2)
    return [new_mean, new_var]

def predict(mean1, var1, mean2, var2):
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]

measurements = [5., 6., 7., 9., 10.]
motion = [1., 1., 2., 1., 1.]
measurement_sig = 4.
motion_sig = 2.

mu = 0.  #初始位置是0,但是很不确定,方差很大
sig = 10000.

#Please print out ONLY the final values of the mean
#and the variance in a list [mu, sig]. 

# Insert code here

for i in range(len(motion)):
    
    mu,sig=update(measurements[i],measurement_sig,mu,sig) #先根据sensor测量分布和先验分布得到估计值,这一步是高斯相乘.(对同一个值,有2种分布,则使用高斯相乘)
    print "update  : ",[mu, sig]
    mu,sig=predict(motion[i],motion_sig,mu,sig)           #机器人移动,移动均值相加,方差相加,这一步是高斯相加 .(对不同值的高斯分布)
    print "predict : ",[mu, sig]

# [5.998000799680128, 5.998400639744102]
# [6.999200191953932, 4.399744061425258]
# [8.999619127420921, 4.09518005751176]
# [9.999811802788143, 4.023515241621696]
# [10.999906177177365, 4.005861580844194]



#print [mu, sig]
#[10.999906177177365, 4.005861580844194]



# update  :  [4.998000799680128, 3.9984006397441023] 根据先验分布和测量分布,(2者都具有不确定性,)高斯相乘:估计此时位置是4.998,可见系统此时更相信sensor,偏向于测量值
# predict :  [5.998000799680128, 5.998400639744102]  移动机器人,移动之前分布已知(系统认为已知),  高斯相加:估计此时位置是5.998
# update  :  [5.999200191953932, 2.399744061425258]  测量后,方差下降,对位置的估计更确信
# predict :  [6.999200191953932, 4.399744061425258]  运动后方差增加,位置的不确定性增加
# update  :  [6.999619127420922, 2.0951800575117594]
# predict :  [8.999619127420921, 4.09518005751176]
# update  :  [8.999811802788143, 2.0235152416216957]
# predict :  [9.999811802788143, 4.023515241621696]
# update  :  [9.999906177177365, 2.0058615808441944]
# predict :  [10.999906177177365, 4.005861580844194]

#思考,sig=0.000000001之后.重新运行上面程序,结果如何解释?