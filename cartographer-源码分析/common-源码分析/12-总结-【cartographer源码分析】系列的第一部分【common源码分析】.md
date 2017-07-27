

 
### 【cartographer源码分析】系列的第一部分【common源码分析】很早就已经完毕。
现总结common文件夹涉及到的主要功能：

## 【1】 common/port.h：
- int RoundToInt(const float x); //四舍五入取整
- FastGzipString(const string& uncompressed, string* compressed);  //压缩字符串string
- FastGunzipString(const string& compressed, string* decompressed); //解压缩字符串


## 【2】common/time.h
主要功能是提供时间转换函数，UniversalTimeScaleClock类实现c++11的clock接口，以0.1us为时间精度。
定义2个别名：
using Duration = UniversalTimeScaleClock::duration;//微秒,0.1us
using Time = UniversalTimeScaleClock::time_point;  //时间点
并提供多个函数方便时间转换.

-  FromSeconds(double seconds); //秒数seconds转为c++的duration实例对象：
-  FromMilliseconds(int64 milliseconds)；
-  ToSeconds(Duration duration);   //将duration实例对象转为秒数s 
-  ToSeconds(Duration duration);   
-  ToUniversal(Time time);将c++的time_point对象转为TUC时间,单位是0.1us

port.h重载了<<操作符,方便将time_point以string输出
std::ostream& operator<<(std::ostream& os, Time time); 
 
## 【3】common/fixed_ratio_sampler.h
该文件定义了FixedRatioSampler类。FixedRatioSampler是频率固定的采样器类，目的是从数据流中均匀的按照固定频率采样数据，提供2个成员函数：

- Pulse()产生一个事件pulses,并且:如果计入采样samples，返回true
- DebugString():以string形式输出采样率

## 【4】common/rate_timer.h

定义了 RateTimer-脉冲频率计数类,作用是计算在一段时间内的脉冲率

 * RateTimer只有一个构造函数RateTimer(const common::Duration ),提供时间段Duration
 * ComputeRate()返回事件脉冲率,单位hz
 * ComputeWallTimeRateRatio()返回真实时间与墙上挂钟时间的比率
 * 内部类Event封装了某个事件发生的时间点.
 * 调用一次Pulse()即产生了一次事件
 *  DebugString() 以字符串形式输出
 *  ComputeDeltasInSeconds() 计算以秒计的delta序列
 *  DeltasDebugString()计算delta序列的均值和方差
 
## 【5】 文件:common/histogram.h
Histogram:直方图类
提供2个操作:
1，Add()//添加value,可乱序
2，ToString(int buckets )以字符串的形式输出buckets个直方图信息,bin大小是篮子个数，表示分为几块统计

## 【6】common/math.h
common/math.h文件主要实现数学计算，包括：区间截断.求n次方.求平方.幅度角度转换.归一化.求反正切值

-  Clamp(const T value, const T min, const T max)；//将val截取到区间min至max中.
-  Power(T base, int exponent)；//计算base的exp次方
-  double DegToRad(double deg)；//.角度到弧度的转换. 60° -> pi/3
-  double RadToDeg(double rad)；//弧度到角度的转换, pi/3 -> 60°
-  NormalizeAngleDifference();  //将角度差转换为[-pi;pi]
-  atan2(const Eigen::Matrix<T, 2, 1>& vector);//以弧度表示点(1,1)的反正切值,范围是[-pi,pi]


## 【7】common/make_unique.h
make_unique.h在不支持c++14的环境下实现 std::make_unique的功能.
实现细节:完美转发和移动语义

## 【8】common/mutex.h
 
common/mutex.h主要是对c++11 的mutex的封装。Mutex类有一个内部类Locker。

- Locker 是一个RAII的类型.本质是使用std::unique_lock和std::condition_variable实现的
   * 在构造Locker时,对mutex上锁,在析构Locker时对mutex解锁.并提供2个成员函数：
    * Await(Predicate predicate)（）
    * AwaitWithTimeout(Predicate predicate, common::Duration timeout)

    功能是利用c++11的条件变量和unique_lock实现在谓词predicate为真的情况下对mutex解锁。

## 【9】common/thread_pool.h
ThreadPool 是对c++11 thread的封装，线程数量固定的线程池类

- 构造函数ThreadPool(int num_threads) 初始化一个线程数量固定的线程池。
- Schedule(std::function<void()> work_item)添加想要ThreadPool执行的函数,   std::thread会在线程后台依次排队执行相关函数.
- 数据成员pool_是具体的线程，work_queue_是待执行的函数队列。
- DoWork()是私有的成员函数，用于具体执行函数任务：每个线程都要执行这个函数.ThreadPool利用条件变量通知此函数,work_queue_不为零时,说明有需要执行的函数,此时加入执行。

## 【10】common/blocking_queue.h
BlockingQueue类是线程安全的阻塞队列,(生产者消费者模式)

 * 构造函数BlockingQueue()初始化队列大小,kInfiniteQueueSize=0默认不限制容量。queue_size限制容量：通过条件变量做到.
 * Push()添加元素,容量不够时,阻塞等待
 * Pop()删除元素,没有元素时,阻塞等待
 * Peek()返回下一个应该弹出的元素
 * PushWithTimeout(),添加元素,若超时则返回false
 * PopWithTimeout(),删除元素，若超时则返回false
 .

关于common文件的源码分析已经完毕，更详细细节可https://github.com/learnmoreonce/cartographer 查看注释版源码。




本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode
