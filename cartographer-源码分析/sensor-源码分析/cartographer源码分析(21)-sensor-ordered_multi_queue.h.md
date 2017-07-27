
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:sensor/ordered_multi_queue.h


#ifndef CARTOGRAPHER_SENSOR_ORDERED_MULTI_QUEUE_H_
#define CARTOGRAPHER_SENSOR_ORDERED_MULTI_QUEUE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>

#include "cartographer/common/blocking_queue.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/data.h"

 
namespace cartographer {
namespace sensor {

struct QueueKey {
  int trajectory_id;// 轨线id;
  string sensor_id; //传感器id

//重载小于运算符,forward_as_tuple:完美转发. (以tuple规则比较2者),tuple定义了<运算符
  bool operator<(const QueueKey& other) const {
    return std::forward_as_tuple(trajectory_id, sensor_id) <
           std::forward_as_tuple(other.trajectory_id, other.sensor_id);
  }
};


/*

OrderedMultiQueue，用于管理多个有序的传感器数据，
是有序的多队列类,每个队列有一个key,并且有一个自定义排序函数
queues_的形式为：
key1:Queue
key2：Queue
key3：Queue

Queue的形式为
  struct Queue {
    common::BlockingQueue<std::unique_ptr<Data>> queue;
    Callback callback;
    bool finished = false;
  };

OrderedMultiQueue的数据成员有
1,common_start_time_per_trajectory_:轨迹id及对应创建轨迹时间
2,last_dispatched_time_
3,std::map<QueueKey, Queue> queues_;按照key排序的map
4,QueueKey blocker_;

*/
// Maintains multiple queues of sorted sensor data and dispatches(迅速办理) it in merge
// sorted order. It will wait to see at least one value for each unfinished
// queue before dispatching the next time ordered value across all queues.
//
// This class is thread-compatible.
class OrderedMultiQueue {
 public:
  using Callback = std::function<void(std::unique_ptr<Data>)>;//回调函数

  OrderedMultiQueue();
  ~OrderedMultiQueue();


//添加一个【队列】Queue,名称是key,以后入队的data，调用回调函数callback处理
  // Adds a new queue with key 'queue_key' which must not already exist.
  // 'callback' will be called whenever data from this queue can be dispatched.
  void AddQueue(const QueueKey& queue_key, Callback callback);

/*
某一key标识的【队列】Queue已经完成入队,因此不能再入队列,并在map中移除key.
*/
  // Marks a queue as finished, i.e. no further data can be added. The queue
  // will be removed once the last piece of data from it has been dispatched.
  void MarkQueueAsFinished(const QueueKey& queue_key);

//对某一key标识的队列Queue,压入data,data按照回调函数处理
  // Adds 'data' to a queue with the given 'queue_key'. Data must be added
  // sorted per queue.
  void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);

/*标记全部队列都已经finished.
 kFirst: {0,4,5,6}
kSecond:{0,1,3,7}
kThird: {0,2,8}
之前只处理到6，调用Flush()则处理剩余的7,8

如果不调用Flush()，则析构时会出错
 */

  // Dispatches all remaining values in sorted order and removes the underlying
  // queues.
  void Flush();

/*返回阻塞的队列(意为该队列对应的sensor的data未到达)*/
  // Must only be called if at least one unfinished queue exists. Returns the
  // key of a queue that needs more data before the OrderedMultiQueue can
  // dispatch data.
  QueueKey GetBlocker() const;

 private:
  struct Queue {
    common::BlockingQueue<std::unique_ptr<Data>> queue;
    Callback callback;
    bool finished = false;
  };

  void Dispatch();
  void CannotMakeProgress(const QueueKey& queue_key);
  common::Time GetCommonStartTime(int trajectory_id);

  // Used to verify that values are dispatched in sorted order.
  common::Time last_dispatched_time_ = common::Time::min();

  std::map<int, common::Time> common_start_time_per_trajectory_;
  std::map<QueueKey, Queue> queues_;//多队列主体,本类最大的内存占用量
  QueueKey blocker_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_ORDERED_MULTI_QUEUE_H_
/*
queues_:
first 是QueueKey
second 是 Queue。

Queue的形式为
  struct Queue {
    common::BlockingQueue<std::unique_ptr<Data>> queue;
    Callback callback;
    bool finished = false;
  };

*/


```

```
文件:ordered_multi_queue.cc


#include "cartographer/sensor/ordered_multi_queue.h"

#include <algorithm>
#include <sstream>
#include <vector>

#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

namespace {

// Number of items that can be queued up before we log which queues are waiting
// for data.
const int kMaxQueueSize = 500;

}  // namespace

//重载QueueKey的<<输出运算符,友元函数
inline std::ostream& operator<<(std::ostream& out, const QueueKey& key) {
  return out << '(' << key.trajectory_id << ", " << key.sensor_id << ')';
}

OrderedMultiQueue::OrderedMultiQueue() {}

OrderedMultiQueue::~OrderedMultiQueue() {
  for (auto& entry : queues_) {
    CHECK(entry.second.finished);
  }
}

//添加一个关键词是key的队列,并用比较函数Callback排序
void OrderedMultiQueue::AddQueue(const QueueKey& queue_key, Callback callback) {
  CHECK_EQ(queues_.count(queue_key), 0);
  //map.count()相对于[],调用时，是不添加key的。而[]是带有副作用的。c++pr.386
  queues_[queue_key].callback = std::move(callback);
}

void OrderedMultiQueue::MarkQueueAsFinished(const QueueKey& queue_key) {
  auto it = queues_.find(queue_key);
  CHECK(it != queues_.end()) << "Did not find '" << queue_key << "'.";
  auto& queue = it->second;
  CHECK(!queue.finished);//检查状态
  queue.finished = true;//标记本队列已完成,别的数据不能再入队.
  Dispatch();           //调用一次MarkQueueAsFinished()就要调用一次Dispatch()
}

//根据key找到队列,并添加data元素
void OrderedMultiQueue::Add(const QueueKey& queue_key,
                            std::unique_ptr<Data> data) {
  auto it = queues_.find(queue_key);
  if (it == queues_.end()) {//没有key时，警告
    LOG_EVERY_N(WARNING, 1000)
        << "Ignored data for queue: '" << queue_key << "'";
    return;
  }
  it->second.queue.Push(std::move(data));//Queue的queue.push()
  Dispatch();//调用一次Add()就要调用一次Dispatch()
}

//先找到没有finished的队列,然后再对这些队列标记finished.已完成的则不作任何处理
void OrderedMultiQueue::Flush() {
  std::vector<QueueKey> unfinished_queues;
  for (auto& entry : queues_) {
    if (!entry.second.finished) {
      unfinished_queues.push_back(entry.first);
    }
  }
  for (auto& unfinished_queue : unfinished_queues) {
    MarkQueueAsFinished(unfinished_queue);//一个一个的处理。
  }
}

QueueKey OrderedMultiQueue::GetBlocker() const {
  CHECK(!queues_.empty());
  return blocker_;
}

/*
Dispatch()函数，不断的处理来自sensor的数据。按照data采集的时间顺序处理。

kFirst: {1,2,3} finised
kSecond:{}      finised
kThird: {}      finised

*/
void OrderedMultiQueue::Dispatch() {
  while (true) {
    //首先处理的数据，也即最早采集的数据
    const Data* next_data = nullptr;
    Queue* next_queue = nullptr;
    QueueKey next_queue_key;
    //遍历队列中的每一个key：填充上面3个变量值。如果某一key对应的data为空，则直接return
    for (auto it = queues_.begin(); it != queues_.end();) {
      const auto* data = it->second.queue.Peek<Data>();//获取某一队的队首data
      if (data == nullptr) { 
        if (it->second.finished) {//it对应的队列为空且为finished,故删除it对应的key
          queues_.erase(it++);    //map迭代器没有失效?
          continue;
        }
        CannotMakeProgress(it->first);//此时什么也不做。
        return;
      }
      if (next_data == nullptr || data->time < next_data->time) {
      //找到next_data数据:即采集时间最早的数据，理论上应该最先处理它。
        next_data = data;
        next_queue = &it->second;
        next_queue_key = it->first;
      }
      CHECK_LE(last_dispatched_time_, next_data->time)
          << "Non-sorted data added to queue: '" << it->first << "'";
      ++it;
    }


    if (next_data == nullptr) {
      CHECK(queues_.empty());//只有多队列为空，才可能next_data==nullptr
      return;
    }


    // If we haven't dispatched any data for this trajectory yet, fast forward
    // all queues of this trajectory until a common start time has been reached.
    const common::Time common_start_time =
        GetCommonStartTime(next_queue_key.trajectory_id);
    //common_start_time即所有的sensor都开始有data的时间点。

    if (next_data->time >= common_start_time) { //大多数情况，happy case
      // Happy case, we are beyond the 'common_start_time' already.
      last_dispatched_time_ = next_data->time;
      next_queue->callback(next_queue->queue.Pop()); //调用回调函数处理data。
    } else if (next_queue->queue.Size() < 2) {// 罕见
      if (!next_queue->finished) {
        // We cannot decide whether to drop or dispatch this yet.
        CannotMakeProgress(next_queue_key);
        return;
      }
      last_dispatched_time_ = next_data->time;
      next_queue->callback(next_queue->queue.Pop());
    } else {
      // We take a peek at the time after next data. If it also is not beyond
      // 'common_start_time' we drop 'next_data', otherwise we just found the
      // first packet to dispatch from this queue.
      std::unique_ptr<Data> next_data_owner = next_queue->queue.Pop();
      if (next_queue->queue.Peek<Data>()->time > common_start_time) {
        last_dispatched_time_ = next_data->time;
        next_queue->callback(std::move(next_data_owner));
      }
    }
  }
}

void OrderedMultiQueue::CannotMakeProgress(const QueueKey& queue_key) {
  blocker_ = queue_key; //标识该队列Queue已经阻塞
  for (auto& entry : queues_) {//std::map<QueueKey, Queue> 
    if (entry.second.queue.Size() > kMaxQueueSize) {//？
      LOG_EVERY_N(WARNING, 60) << "Queue waiting for data: " << queue_key;
      return;
    }
  }
}

/*对同一轨迹id，求得所有sensor的首次采集data的最晚时间maxtime

不同轨迹按不同轨迹算：
kFirst: {0,1,2,3} finised
kSecond:{2}       
kThird: {4}
{2,2,}

*/
common::Time OrderedMultiQueue::GetCommonStartTime(const int trajectory_id) {
  //map.emplace():Construct and insert element，根据trajectory_id构造一个map。
  auto emplace_result = common_start_time_per_trajectory_.emplace(
      trajectory_id, common::Time::min());
  common::Time& common_start_time = emplace_result.first->second;
  if (emplace_result.second) {
    for (auto& entry : queues_) { 
    //entry是map的pair<,>.本循环求得所有传感器中的maxtime
      if (entry.first.trajectory_id == trajectory_id) {
        common_start_time =
            std::max(common_start_time, entry.second.queue.Peek<Data>()->time);
      }
    }
    LOG(INFO) << "All sensor data for trajectory " << trajectory_id
              << " is available starting at '" << common_start_time << "'.";
  }
  return common_start_time;
}

}  // namespace sensor
}  // namespace cartographer


/*
ordered_multi_queue.cc:172] All sensor data for trajectory 0 is available starting at '635727208200176089'

*/

/*
LOG_EVERY_N(INFO, 60) << "Queue...";在程序中周期性的记录日志信息，
在该语句第1、61、121……次被执行的时候，记录日志信息。
*/

```

```c++
测试代码:ordered_multi_queue_le_test.cc


#define sout(Xit)  {std::cout<<__LINE__<<" "<< Xit <<""<<std::endl;}

#include "cartographer/sensor/ordered_multi_queue.h"

#include <vector>

#include "cartographer/common/make_unique.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace sensor {
namespace {
/*测试夹具（Test Fixtures）
TEST_F提供了一个初始化函数（SetUp）和一个清理函数(TearDown)，
在TEST_F中使用的变量可以在初始化函数SetUp中初始化，
在TearDown中销毁，并且所有的TEST_F是互相独立的，都是在初始化以后的状态开始运行，
一个TEST_F不会影响另一个TEST_F所使用的数据.
用TEST_F定义测试，写法与TEST相同，但测试用例名必须为上面定义的类名。
*/
class OrderedMultiQueueTest : public ::testing::Test {
 protected:
  // These are keys are chosen so that they sort first, second, third.
  const QueueKey kFirst{1, "a"};//trajectory_id，sensor_id
  const QueueKey kSecond{1, "b"};
  const QueueKey kThird{2, "b"};

//void AddQueue(const QueueKey& queue_key, Callback callback);
  void SetUp() {
  //初始化函数,定义处理data的函数：
  //(以后用于test比较):检查values_的最大值是否小于欲添加的值.
    for (const auto& queue_key : {kFirst, kSecond, kThird}) {
      queue_.AddQueue(queue_key, [this](std::unique_ptr<Data> data) {
        if (!values_.empty()) {
          EXPECT_GE(data->time, values_.back().time);
        }
        values_.push_back(*data);
      });
    }
  }

//Data的定义在data.h
  std::unique_ptr<Data> MakeImu(const int ordinal) {//时间,序列
    return common::make_unique<Data>(
        common::FromUniversal(ordinal),
        Data::Imu{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});
  }

  std::vector<Data> values_;
  OrderedMultiQueue queue_;
};


TEST_F(OrderedMultiQueueTest, Ordering) {
  // void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);
  queue_.Add(kFirst, MakeImu(0));//
  queue_.Add(kFirst, MakeImu(4));
  queue_.Add(kFirst, MakeImu(5));
  queue_.Add(kFirst, MakeImu(6));
  EXPECT_TRUE(values_.empty());  
  //why为空?：queue_对应的其他sensor还没有产生data，
  //故调用Add()时，只在本队列插入data，而Dispatch()直接返回，没有调用Callback函数。


  queue_.Add(kSecond, MakeImu(0));
  queue_.Add(kSecond, MakeImu(1));
  EXPECT_TRUE(values_.empty()); //同上


  queue_.Add(kThird, MakeImu(0));
  queue_.Add(kThird, MakeImu(2));
  EXPECT_EQ(values_.size(), 4);

 /*
kFirst: {0,4,5,6}
kSecond:{0,1}
kThird: {0,2}

*/
  for(auto i:values_)
  {
      sout(i.time);//{0,0,0,1}
  }


  queue_.Add(kSecond, MakeImu(3));
  EXPECT_EQ(values_.size(), 5);
/*
kFirst: {0,4,5,6}
kSecond:{0,1,3}
kThird: {0,2}

*/
  for(auto i:values_)
  {
      sout(i.time);//{0,0,0,1,2}
  }

  queue_.Add(kSecond, MakeImu(7));
  queue_.Add(kThird, MakeImu(8));

  /*
kFirst: {0,4,5,6}
kSecond:{0,1,3,7}
kThird: {0,2,8}

*/
  for(auto i:values_)
  {
      sout(i.time);//{0,0,0,1,2,3,4,5,6}
  }


//先找到没有finished的队列,然后再对这些队列标记finished
  queue_.Flush();//应该调用,否则析构时出错
  EXPECT_EQ(11, values_.size());// 4+4+3

    /*
kFirst: {0,4,5,6}
kSecond:{0,1,3,7}
kThird: {0,2,8}

*/
  for(auto i:values_)
  {
      sout(i.time);//{0,0,0,1,2,3,4,5,6,7,8}
  }
  for (size_t i = 0; i < values_.size() - 1; ++i) { //检查是否按序
    EXPECT_LE(values_[i].time, values_[i + 1].time);
  }
}

TEST_F(OrderedMultiQueueTest, MarkQueueAsFinished) {
  queue_.Add(kFirst, MakeImu(1));
  queue_.Add(kFirst, MakeImu(2));
  queue_.Add(kFirst, MakeImu(3));
  EXPECT_TRUE(values_.empty());
  queue_.MarkQueueAsFinished(kFirst);

//调用该函数一次，就调用一次Dispatch(),若key对应的队列为空，则从队列中删除key
  EXPECT_TRUE(values_.empty());
  queue_.MarkQueueAsFinished(kSecond);
  EXPECT_TRUE(values_.empty());
  queue_.MarkQueueAsFinished(kThird);

    /*
kFirst: {1,2,3} finised
kSecond:{}      finised
kThird: {}      finised

*/
  for(auto i:values_)
  {
      sout(i.time);//{1,2,3}
  }

  EXPECT_EQ(3, values_.size());
  for (size_t i = 0; i < values_.size(); ++i) {
    EXPECT_EQ(i + 1, common::ToUniversal(values_[i].time));
  }
}

TEST_F(OrderedMultiQueueTest, CommonStartTimePerTrajectory) {
  queue_.Add(kFirst, MakeImu(0));
  queue_.Add(kFirst, MakeImu(1));
  queue_.Add(kFirst, MakeImu(2));
  queue_.Add(kFirst, MakeImu(3));
  queue_.Add(kSecond, MakeImu(2));
  EXPECT_TRUE(values_.empty());
  queue_.Add(kThird, MakeImu(4));
  EXPECT_EQ(values_.size(), 2);
/*
kFirst: {0,1,2,3}
kSecond:{2}
kThird: {4}
*/
  for(auto i:values_)
  {
      sout(i.time);//{2,2}
  }


  queue_.MarkQueueAsFinished(kFirst);
  EXPECT_EQ(values_.size(), 2);//没有0,1，因为起点时间不是0,1，而是2
/*
kFirst: {0,1,2,3} finised
kSecond:{2}       
kThird: {4}
*/
  for(auto i:values_)
  {
      sout(i.time);//{2,2}
  }

  sout(".id2.");//为何与test1不同？起点time不同。test1：起点时间全为0，test3起点时间是2
  queue_.MarkQueueAsFinished(kSecond);
  EXPECT_EQ(values_.size(), 4);
/*
kFirst: {0,1,2,3} finised
kSecond:{2}       finised
kThird: {4}
*/
  for(auto i:values_)
  {
      sout(i.time);//{2,2,3,4}
  }

//第二条轨迹id开始采集数据。
  queue_.MarkQueueAsFinished(kThird);
  EXPECT_EQ(values_.size(), 4);
/*
kFirst: {0,1,2,3} finised
kSecond:{2}       finised
kThird: {4}       finised
*/
  for(auto i:values_)
  {
      sout(i.time);//{2,2,3,4}
  }
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer


```

本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode
