
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:common/blocking_queue.h


#ifndef CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_
#define CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_

#include <cstddef>
#include <deque>
#include <memory>

#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "glog/logging.h"
/*
 * BlockingQueue是线程安全的阻塞队列,(生产者消费者模式)
 * 不可拷贝不可赋值
 * 构造函数初始化队列大小,kInfiniteQueueSize=0默认不限制容量。queue_size限制容量：通过条件变量做到.
 * Push()添加元素,容量不够时,阻塞等待
 * Pop()删除元素,没有元素时,阻塞等待
 * Peek()返回下一个应该弹出的元素
 * PushWithTimeout(),添加元素,若超时则返回false
 * PopWithTimeout(),删除元素，若超时则返回false
 *
 * */
namespace cartographer {
namespace common {

// A thread-safe blocking queue that is useful for producer/consumer patterns.
// 'T' must be movable.
template <typename T>
class BlockingQueue {
 public:
  static constexpr size_t kInfiniteQueueSize = 0;

  // Constructs a blocking queue with infinite queue size.
  BlockingQueue() : BlockingQueue(kInfiniteQueueSize) {} //默认不限制容量

  BlockingQueue(const BlockingQueue&) = delete;
  BlockingQueue& operator=(const BlockingQueue&) = delete;

  // Constructs a blocking queue with a size of 'queue_size'.限制容量
  explicit BlockingQueue(const size_t queue_size) : queue_size_(queue_size) {}

  // Pushes a value onto the queue. Blocks if the queue is full.
  void Push(T t) {
    MutexLocker lock(&mutex_);//加锁
    lock.Await([this]() REQUIRES(mutex_) { return QueueNotFullCondition(); });//队列未满才能push
    deque_.push_back(std::move(t)); //移动而非拷贝
  }

  // Like push, but returns false if 'timeout' is reached.
  bool PushWithTimeout(T t, const common::Duration timeout) { //最多等待timeout时间
    MutexLocker lock(&mutex_);
    if (!lock.AwaitWithTimeout(
            [this]() REQUIRES(mutex_) { return QueueNotFullCondition(); },
            timeout)) {
      return false;
    }
    deque_.push_back(std::move(t));
    return true;
  }

  // Pops the next value from the queue. Blocks until a value is available.
  T Pop() {
    MutexLocker lock(&mutex_); //加锁
    lock.Await([this]() REQUIRES(mutex_) { return QueueNotEmptyCondition(); });//队列不为空才能pop()

    T t = std::move(deque_.front());
    deque_.pop_front();
    return t;
  }

  // Like Pop, but can timeout. Returns nullptr in this case.
  T PopWithTimeout(const common::Duration timeout) {
    MutexLocker lock(&mutex_);
    if (!lock.AwaitWithTimeout(
            [this]() REQUIRES(mutex_) { return QueueNotEmptyCondition(); },
            timeout)) {
      return nullptr;
    }
    T t = std::move(deque_.front());
    deque_.pop_front();
    return t;
  }

  // Returns the next value in the queue or nullptr if the queue is empty.
  // Maintains ownership. This assumes a member function get() that returns
  // a pointer to the given type R.
  template <typename R>
  const R* Peek() {
    MutexLocker lock(&mutex_);
    if (deque_.empty()) {
      return nullptr;
    }
    return deque_.front().get();
  }

  // Returns the number of items currently in the queue.
  size_t Size() {
    MutexLocker lock(&mutex_);
    return deque_.size();
  }

 private:
  // Returns true iff the queue is not empty.
  bool QueueNotEmptyCondition() REQUIRES(mutex_) { return !deque_.empty(); }

  // Returns true iff the queue is not full.
  bool QueueNotFullCondition() REQUIRES(mutex_) {
    return queue_size_ == kInfiniteQueueSize || deque_.size() < queue_size_;
  }

  Mutex mutex_;
  const size_t queue_size_ GUARDED_BY(mutex_);
  std::deque<T> deque_ GUARDED_BY(mutex_);
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_



```


```c++
代码blocking_queue.cc:

/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/common/blocking_queue.h"

#include <memory>
#include <thread>

#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

TEST(BlockingQueueTest, testPushPeekPop) {
  BlockingQueue<std::unique_ptr<int>> blocking_queue;//元素是智能指针,不限容
  blocking_queue.Push(common::make_unique<int>(42)); //添加
  ASSERT_EQ(1, blocking_queue.Size());               //size==1
  blocking_queue.Push(common::make_unique<int>(24)); //添加
  ASSERT_EQ(2, blocking_queue.Size());               //size==2
  EXPECT_EQ(42, *blocking_queue.Peek<int>());        //顶元素==42
  ASSERT_EQ(2, blocking_queue.Size());               //队列大小是2
  EXPECT_EQ(42, *blocking_queue.Pop());
  ASSERT_EQ(1, blocking_queue.Size());
  EXPECT_EQ(24, *blocking_queue.Pop());
  ASSERT_EQ(0, blocking_queue.Size());
  EXPECT_EQ(nullptr, blocking_queue.Peek<int>());
  ASSERT_EQ(0, blocking_queue.Size());
}

TEST(BlockingQueueTest, testPushPopSharedPtr) {
  BlockingQueue<std::shared_ptr<int>> blocking_queue;  //不限容
  blocking_queue.Push(std::make_shared<int>(42));
  blocking_queue.Push(std::make_shared<int>(24));
  EXPECT_EQ(42, *blocking_queue.Pop());
  EXPECT_EQ(24, *blocking_queue.Pop());
}

TEST(BlockingQueueTest, testPopWithTimeout) {
  BlockingQueue<std::unique_ptr<int>> blocking_queue;
  EXPECT_EQ(nullptr,                                 //没有插入,所以是nullptr
            blocking_queue.PopWithTimeout(common::FromMilliseconds(150)));
}

TEST(BlockingQueueTest, testPushWithTimeout) {
  BlockingQueue<std::unique_ptr<int>> blocking_queue(1);//限容
  EXPECT_EQ(true,                                   //插入成功
            blocking_queue.PushWithTimeout(common::make_unique<int>(42),
                                           common::FromMilliseconds(150)));
  EXPECT_EQ(false,                                  //失败,因为队列大小是1
            blocking_queue.PushWithTimeout(common::make_unique<int>(15),
                                           common::FromMilliseconds(150)));
  EXPECT_EQ(42, *blocking_queue.Pop());
  EXPECT_EQ(0, blocking_queue.Size());
}

TEST(BlockingQueueTest, testPushWithTimeoutInfinteQueue) {
  BlockingQueue<std::unique_ptr<int>> blocking_queue; //不限容量
  EXPECT_EQ(true,
            blocking_queue.PushWithTimeout(common::make_unique<int>(42),
                                           common::FromMilliseconds(150)));
  EXPECT_EQ(true,
            blocking_queue.PushWithTimeout(common::make_unique<int>(45),
                                           common::FromMilliseconds(150)));
  EXPECT_EQ(42, *blocking_queue.Pop());
  EXPECT_EQ(45, *blocking_queue.Pop());
  EXPECT_EQ(0, blocking_queue.Size());
}

TEST(BlockingQueueTest, testBlockingPop) {
  BlockingQueue<std::unique_ptr<int>> blocking_queue;
  ASSERT_EQ(0, blocking_queue.Size());

  int pop = 0;
  //多线程测试
  std::thread thread([&blocking_queue, &pop] { pop = *blocking_queue.Pop(); });

  std::this_thread::sleep_for(common::FromMilliseconds(100));
  blocking_queue.Push(common::make_unique<int>(42));
  thread.join();
  ASSERT_EQ(0, blocking_queue.Size());
  EXPECT_EQ(42, pop);
}

TEST(BlockingQueueTest, testBlockingPopWithTimeout) {
  BlockingQueue<std::unique_ptr<int>> blocking_queue;
  ASSERT_EQ(0, blocking_queue.Size());

  int pop = 0;
  //多线程测试
  std::thread thread([&blocking_queue, &pop] {
    pop = *blocking_queue.PopWithTimeout(common::FromMilliseconds(2500));
  });

  std::this_thread::sleep_for(common::FromMilliseconds(100));
  blocking_queue.Push(common::make_unique<int>(42));
  thread.join();
  ASSERT_EQ(0, blocking_queue.Size());
  EXPECT_EQ(42, pop);
}

}  // namespace
}  // namespace common
}  // namespace cartographer


```

本文发于：
*  http://www.jianshu.com/u/9e38d2febec1 （推荐）
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce