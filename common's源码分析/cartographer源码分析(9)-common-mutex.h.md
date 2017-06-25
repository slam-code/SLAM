
源码可在https://github.com/learnmoreonce/SLAM 下载

``` c++
 
文件:common/mutex.h



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
Mutex是c++11 mutex的封装
不使用clang时,宏定义全部为空操作.故看见宏定义时可暂时忽略
*/


// Defines an annotated mutex that can only be locked through its scoped locker
// implementation.
class CAPABILITY("mutex") Mutex {
 public:

 /*
Locker 是一个RAII的类型.在用mutex构造Locker时,对mutex上锁,在析构Locker时对mutex解锁.
本质是使用std::unique_lock和std::condition_variable
 */

  // A RAII class that acquires a mutex in its constructor, and
  // releases it in its destructor. It also implements waiting functionality on
  // conditions that get checked whenever the mutex is released.
  class SCOPED_CAPABILITY Locker {
   public:
    Locker(Mutex* mutex) ACQUIRE(mutex) : mutex_(mutex), lock_(mutex->mutex_) {}

    ~Locker() RELEASE() {
      lock_.unlock();
      mutex_->condition_.notify_all();
    }

    template <typename Predicate>
    void Await(Predicate predicate) REQUIRES(this) {
      mutex_->condition_.wait(lock_, predicate);
    }

    template <typename Predicate>
    bool AwaitWithTimeout(Predicate predicate, common::Duration timeout)
        REQUIRES(this) {
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



```


```c++
测试代码:


```