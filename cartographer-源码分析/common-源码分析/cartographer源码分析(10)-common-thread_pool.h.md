
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:common/thread_pool.h



#ifndef CARTOGRAPHER_COMMON_THREAD_POOL_H_
#define CARTOGRAPHER_COMMON_THREAD_POOL_H_

#include <deque>
#include <functional>
#include <thread>
#include <vector>

#include "cartographer/common/mutex.h"

namespace cartographer {
namespace common {

/*
ThreadPool 是对c++11 thread的封装.
ThreadPool是线程数量固定的线程池，不可拷贝 和复制.

1，构造函数ThreadPool(int num_threads) 初始化一个线程数量固定的线程池。
2，Schedule(std::function<void()> work_item)添加想要ThreadPool执行的函数,
   std::thread会在线程后台依次排队执行相关函数.
3，数据成员pool_是具体的线程，work_queue_是待执行的函数队列。


*/
// A fixed number of threads working on a work queue of work items. Adding a
// new work item does not block, and will be executed by a background thread
// eventually. The queue must be empty before calling the destructor. The thread
// pool will then wait for the currently executing work items to finish and then
// destroy the threads.
class ThreadPool {
 public:
  explicit ThreadPool(int num_threads);
  ~ThreadPool();

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  void Schedule(std::function<void()> work_item);

 private:
  void DoWork();

  Mutex mutex_; //互斥锁，保证线程安全

  //running_只是一个监视哨,只有线程池在running_状态时,才能往work_queue_加入函数.
  bool running_ GUARDED_BY(mutex_) = true;     
  std::vector<std::thread> pool_ GUARDED_BY(mutex_);
  std::deque<std::function<void()>> work_queue_ GUARDED_BY(mutex_);
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_THREAD_POOL_H_



```


```c++

common/thread_pool.cc:


#include "cartographer/common/thread_pool.h"

#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <numeric>

#include "glog/logging.h"

namespace cartographer {
namespace common {

/*
pool_是vector,每个线程初始化时,执行DoWork()函数.

*/
ThreadPool::ThreadPool(int num_threads) {
  MutexLocker locker(&mutex_);
  for (int i = 0; i != num_threads; ++i) {
    pool_.emplace_back([this]() { ThreadPool::DoWork(); });
  }
}

/*
只有线程池的work_queue_=0时,才能析构,此时所有的线程已经开始被pool执行,
只有等待pool结束所有的线程执行函数(join结束),ThreadPool才能析构完成
*/
ThreadPool::~ThreadPool() {
  {
    MutexLocker locker(&mutex_);
    CHECK(running_);
    running_ = false;
    CHECK_EQ(work_queue_.size(), 0);
  }
  for (std::thread& thread : pool_) {
    thread.join();
  }
}

/*
使想要线程池执行的work_item函数加入到等待队列中,然后排队等待空闲线程"领取"该函数.
*/
void ThreadPool::Schedule(std::function<void()> work_item) {
  MutexLocker locker(&mutex_);
  CHECK(running_);
  work_queue_.push_back(work_item);
}

/*
每个线程都要执行这个函数.
ThreadPool利用条件变量通知此函数,work_queue_不为零时,说明有需要执行的函数,此时加入执行
*/
void ThreadPool::DoWork() {
#ifdef __linux__
  // This changes the per-thread nice level of the current thread on Linux. We
  // do this so that the background work done by the thread pool is not taking
  // away CPU resources from more important foreground threads.
  CHECK_NE(nice(10), -1);
#endif
  for (;;) {
    std::function<void()> work_item;
    {
      //加锁,同一时刻只能一个线程"领取"某个函数,何时执行这个函数呢? 
      // 最后一行即为抢到的线程开始执行work_item
      MutexLocker locker(&mutex_);
      locker.Await([this]() REQUIRES(mutex_) {
        return !work_queue_.empty() || !running_;   
        //注意,队列不为空或者不在running_状态,条件变量都需要通知该函数
      });
      if (!work_queue_.empty()) {
        work_item = work_queue_.front();    //领取
        work_queue_.pop_front();            //删除
      } else if (!running_) { 
        return;
      }
    }
    CHECK(work_item);
    work_item();
  }
}

}  // namespace common
}  // namespace cartographer



```

本文发于：
*  http://www.jianshu.com/u/9e38d2febec1 （推荐）
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce