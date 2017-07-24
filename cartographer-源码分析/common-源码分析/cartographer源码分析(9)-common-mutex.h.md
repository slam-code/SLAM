
源码可在https://github.com/learnmoreonce/SLAM 下载

``` c++
 
文件:common/mutex.h


/*

common/mutex.h主要是对c++11 的mutex的封装。

*/
#ifndef CARTOGRAPHER_COMMON_MUTEX_H_
#define CARTOGRAPHER_COMMON_MUTEX_H_

#include <condition_variable>
#include <mutex>

#include "cartographer/common/time.h"

namespace cartographer {
namespace common {

/*不使用clang时,以下宏定义全部为空操作.故看见宏定义时可暂时忽略*/
// Enable thread safety attributes only with clang.
// The attributes can be safely erased when compiling with other compilers.
#if defined(__SUPPORT_TS_ANNOTATION__) || defined(__clang__)
#define THREAD_ANNOTATION_ATTRIBUTE__(x) __attribute__((x))
#else
#define THREAD_ANNOTATION_ATTRIBUTE__(x)  // no-op,空操作
#endif

#define CAPABILITY(x) THREAD_ANNOTATION_ATTRIBUTE__(capability(x))

#define SCOPED_CAPABILITY THREAD_ANNOTATION_ATTRIBUTE__(scoped_lockable)

#define GUARDED_BY(x) THREAD_ANNOTATION_ATTRIBUTE__(guarded_by(x))

#define PT_GUARDED_BY(x) THREAD_ANNOTATION_ATTRIBUTE__(pt_guarded_by(x))

#define REQUIRES(...) \
  THREAD_ANNOTATION_ATTRIBUTE__(requires_capability(__VA_ARGS__))

#define ACQUIRE(...) \
  THREAD_ANNOTATION_ATTRIBUTE__(acquire_capability(__VA_ARGS__))

#define RELEASE(...) \
  THREAD_ANNOTATION_ATTRIBUTE__(release_capability(__VA_ARGS__))

#define EXCLUDES(...) THREAD_ANNOTATION_ATTRIBUTE__(locks_excluded(__VA_ARGS__))

#define NO_THREAD_SAFETY_ANALYSIS \
  THREAD_ANNOTATION_ATTRIBUTE__(no_thread_safety_analysis)

/*
Mutex是c++11 mutex的封装，
Mutex类有一个内部类Locker。

Locker 是一个RAII的类型.
在构造Locker时,对mutex上锁,在析构Locker时对mutex解锁.
本质是使用std::unique_lock和std::condition_variable实现的

Locker类提供2个成员函数Await()和AwaitWithTimeout()
功能是利用c++11的条件变量和unique_lock实现在谓词predicate为真的情况下对mutex解锁。
*/


// Defines an annotated mutex that can only be locked through its scoped locker
// implementation.
class CAPABILITY("mutex") Mutex {
 public:

  // A RAII class that acquires a mutex in its constructor, and
  // releases it in its destructor. It also implements waiting functionality on
  // conditions that get checked whenever the mutex is released.
  class SCOPED_CAPABILITY Locker {
   public:
    Locker(Mutex* mutex) ACQUIRE(mutex) : mutex_(mutex), lock_(mutex->mutex_) {}//上锁

    ~Locker() RELEASE() {
      lock_.unlock();                 //解锁
      mutex_->condition_.notify_all();//条件变量通知解锁
    }

    template <typename Predicate>
    void Await(Predicate predicate) REQUIRES(this) { 
      /*wait()实质是分3步：
      1.对lock_解锁
      2.等待predicate谓词为真，此时调用端阻塞。
      3.对lock_ 重新上锁。
      */
      mutex_->condition_.wait(lock_, predicate);
    }

    template <typename Predicate>
    bool AwaitWithTimeout(Predicate predicate, common::Duration timeout)
        REQUIRES(this) {       //等待谓词为真或者直到超时timeout返回。
      return mutex_->condition_.wait_for(lock_, timeout, predicate);
    }

   private:
    Mutex* mutex_;
    std::unique_lock<std::mutex> lock_;
  };

 private:
  std::condition_variable condition_;
  std::mutex mutex_;
};

using MutexLocker = Mutex::Locker;

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_MUTEX_H_

/*
谓词.断言.Predicate:
一个用于比较的函数（或函数对象），输入两个T类型，返回bool类型。
*/

```


```c++
测试代码:


```


本文发于：
*  http://www.jianshu.com/u/9e38d2febec1  
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode