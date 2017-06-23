
例子讲解:
    http://blog.sciencenet.cn/blog-3276147-1023387.html
    http://blog.sciencenet.cn/blog-3276147-1030202.html
    http://blog.csdn.net/liminlu0314/article/details/15887371
    http://blog.csdn.net/liminlu0314/article/details/16808239
翻译:
    http://blog.csdn.net/liminlu0314/article/details/15860677
    http://blog.csdn.net/yizhou2010/article/details/52712202
    http://blog.csdn.net/yizhou2010/article/details/52749890
    http://blog.csdn.net/yizhou2010/article/details/52618968
1、先略读教材：王宜举, 修乃华. 非线性最优化理论与方法[J]. 2012.
    再依据Ceres Solver官网教程进行学习http://www.ceres-solver.org/tutorial.html

3、tutorial例子解析

例3.1 examples/helloworld.cc

使函数
        y=0.5 (10 - x)^2
最小化

第一步：利用ceres库的模板，创新CostFunctor；f(x)=10-x;

第二步：建立Problem，把CostFunctor转换为用于ceres库解算的残差块形式

             使用class AutoDiffCostFunction(对f(x)自动求导)

             problem.AddResidualBlock（添加到残差块）

第三步：声明解算器、控制器

             Solver::Optionsoptions;

             options.linear_solver_type=ceres::DENSE_QR;

options.minimizer_progress_to_stdout=true;

第四步：解算过程报告输出

             Solver::Summarysummary;

 Solve(options,&problem,&summary);

运行结果如下：
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  4.512500e+01    0.00e+00    9.50e+00   0.00e+00   0.00e+00  1.00e+04        0    3.35e-05    6.78e-04
   1  4.511598e-07    4.51e+01    9.50e-04   9.50e+00   1.00e+00  3.00e+04        1    7.26e-04    1.49e-03
   2  5.012552e-16    4.51e-07    3.17e-08   9.50e-04   1.00e+00  9.00e+04        1    2.46e-05    1.53e-03
Ceres Solver Report: Iterations: 3, Initial cost: 4.512500e+01, Final cost: 5.012552e-16, Termination: CONVERGENCE
x : 0.5 -> 10

经过三次迭代计算，求得的x值为10时可以取得最小值。



**********例3.1-3.6小结***********

①以上例子均使用的迭代优化，即需要初值

但对于例3.1的1/2*(10-x)^2的最小值，导数=0时x=10可直接求出。但对大多数优化情况中，cost函数比较复杂，导数形式无法解析或导数=0不易求解，这时我们大多使用迭代方法求解。

②默认生成的代价函数被二范数化

例3.1中，定义的CostFunctor为 f(x)=10-x;然后在problem环节中，使用AutoDiffCostFunction生成的CostFunction 为1/2* ||f(x)||^2。因为优化框架会默认进行二范数化。

③例3.2和3.3 numeric derivatives（数值微分）和Analytic Derivatives（解析求导）。

数值微分是简单的形式是有限差分法。Ceres也提供了三种有限差分法：

