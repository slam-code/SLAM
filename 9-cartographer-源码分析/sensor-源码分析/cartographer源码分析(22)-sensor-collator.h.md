
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:sensor/collator.h



#ifndef CARTOGRAPHER_SENSOR_COLLATOR_H_
#define CARTOGRAPHER_SENSOR_COLLATOR_H_

#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cartographer/sensor/data.h"
#include "cartographer/sensor/ordered_multi_queue.h"

namespace cartographer {
namespace sensor {

/*
Collator,采集者，校对者，整理者
将多传感器采集的数据归并到轨迹上。

Collator类:不可拷贝,不可赋值.
只有一个默认构造函数.
有2个数据成员

1,OrderedMultiQueue queue_; key是pair<轨迹线id,传感器id>.
一般情况下，对于已有的bag文件，轨迹id等于0.

2,std::unordered_map<int, std::vector<QueueKey>> queue_keys_
轨迹线和队列key组成的hash表,1：N模式



Collator类主要提供三个操作:
1,AddTrajectory() 添加一个轨迹线,
2,FinishTrajectory() 标记轨迹线已经采集完成
3,AddSensorData()接收传感器数据
4,Flush()刷新

*/
class Collator {
 public:
  using Callback = std::function<void(const string&, std::unique_ptr<Data>)>;

  Collator() {}

  Collator(const Collator&) = delete;
  Collator& operator=(const Collator&) = delete;

/*添加一个轨迹线,接受有序的传感器数据,并使用callback回调处理data。
  一个轨迹线对应多个传感器数据:id ->unordered_set
*/
  // Adds a trajectory to produce sorted sensor output for. Calls 'callback'
  // for each collated sensor data.
  void AddTrajectory(int trajectory_id,
                     const std::unordered_set<string>& expected_sensor_ids,
                     Callback callback);

  // Marks 'trajectory_id' as finished.标记轨迹线已经完成采样.
  void FinishTrajectory(int trajectory_id);

/*添加一个传感器id对应的数据data,数据必须按时间排序，*/
  // Adds 'data' for 'trajectory_id' to be collated. 'data' must contain valid
  // sensor data. Sensor packets with matching 'sensor_id' must be added in time
  // order.
  void AddSensorData(int trajectory_id, const string& sensor_id,
                     std::unique_ptr<Data> data);


//刷新
  // Dispatches all queued sensor packets. May only be called once.
  // AddSensorData may not be called after Flush.
  void Flush();


/*返回阻塞的轨迹id,
此种情况多见于某一传感器持久未采集data，
造成ordered_multi_queue阻塞。

  */
  // Must only be called if at least one unfinished trajectory exists. Returns
  // the ID of the trajectory that needs more data before the Collator is
  // unblocked.
  int GetBlockingTrajectoryId() const;

 private:
  //多个key构成的多队列
  // Queue keys are a pair of trajectory ID and sensor identifier.
  OrderedMultiQueue queue_;
  
  //int为传感器id,vector是id+sensor组成的QueueKey
  
  // Map of trajectory ID to all associated QueueKeys.
  std::unordered_map<int, std::vector<QueueKey>> queue_keys_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_COLLATOR_H_


```

```
文件：collator.cc


#include "cartographer/sensor/collator.h"

namespace cartographer {
namespace sensor {

/*
一个轨迹线,多个传感器

*/

/*
struct QueueKey {
  int trajectory_id;// 轨线id;
  string sensor_id; //传感器id
  }

*/
void Collator::AddTrajectory(
    const int trajectory_id,
    const std::unordered_set<string>& expected_sensor_ids,
    const Callback callback) {
  for (const auto& sensor_id : expected_sensor_ids) { 
    //对于每一个轨迹线+传感器,设置一个key
    const auto queue_key = QueueKey{trajectory_id, sensor_id};

    //添加一个名为key的队列,并设置回调函数处理data
    queue_.AddQueue(queue_key,
                    [callback, sensor_id](std::unique_ptr<Data> data) {
                      callback(sensor_id, std::move(data));
                    });

    //map<int,vector<key>>:添加轨迹线对应的key
    queue_keys_[trajectory_id].push_back(queue_key);
  }
}


/*队列不再接收数据*/
void Collator::FinishTrajectory(const int trajectory_id) {
  for (const auto& queue_key : queue_keys_[trajectory_id]) {
    queue_.MarkQueueAsFinished(queue_key);
  }
}


/*
主要的操作,添加传感器数据,数据形式是:key+data
*/
void Collator::AddSensorData(const int trajectory_id, const string& sensor_id,
                             std::unique_ptr<Data> data) {
  //找到key，再move(data)
  queue_.Add(QueueKey{trajectory_id, sensor_id}, std::move(data));
}

void Collator::Flush() { queue_.Flush(); }

int Collator::GetBlockingTrajectoryId() const {
  return queue_.GetBlocker().trajectory_id;
}

}  // namespace sensor
}  // namespace cartographer


```c++
测试代码:collator_test.cc


#include "cartographer/sensor/collator.h"

#include <array>
#include <memory>

#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace sensor {
namespace {

TEST(Collator, Ordering) {
  const std::array<string, 4> kSensorId = {
      {"horizontal_rangefinder", "vertical_rangefinder", "imu", "odometry"}};
  Data zero(common::FromUniversal(0), Data::Rangefinder{});
  Data first(common::FromUniversal(100), Data::Rangefinder{});
  Data second(common::FromUniversal(200), Data::Rangefinder{});
  Data third(common::FromUniversal(300), Data::Imu{});
  Data fourth(common::FromUniversal(400), Data::Rangefinder{});
  Data fifth(common::FromUniversal(500), Data::Rangefinder{});
  Data sixth(common::FromUniversal(600), transform::Rigid3d::Identity());
  //构造6个传感器数据

  std::vector<std::pair<string, Data>> received;
  Collator collator;
  collator.AddTrajectory(                                              
   //添加一个轨迹线0,多个传感器,并指定回调函数:所有的接收数据都存储在received中
      0, 
      std::unordered_set<string>(kSensorId.begin(), kSensorId.end()),

      [&received](const string& sensor_id, std::unique_ptr<Data> data) {
        received.push_back(std::make_pair(sensor_id, *data));
        //由传感器id和data组成一个pair
      });

  constexpr int kTrajectoryId = 0;

  // Establish a common start time.
  collator.AddSensorData(kTrajectoryId, kSensorId[0],
                         common::make_unique<Data>(zero)); 
                         //轨迹0,传感器0产生一个数据,产生时间是0us

  collator.AddSensorData(kTrajectoryId, kSensorId[1],
                         common::make_unique<Data>(zero));

  collator.AddSensorData(kTrajectoryId, kSensorId[2],
                         common::make_unique<Data>(zero));

  collator.AddSensorData(kTrajectoryId, kSensorId[3],
                         common::make_unique<Data>(zero)); //同上

  collator.AddSensorData(kTrajectoryId, kSensorId[0],
                         common::make_unique<Data>(first));//产生数据
  collator.AddSensorData(kTrajectoryId, kSensorId[3],
                         common::make_unique<Data>(sixth));
  collator.AddSensorData(kTrajectoryId, kSensorId[0],
                         common::make_unique<Data>(fourth));
  collator.AddSensorData(kTrajectoryId, kSensorId[1],
                         common::make_unique<Data>(second));
  collator.AddSensorData(kTrajectoryId, kSensorId[1],
                         common::make_unique<Data>(fifth));
  collator.AddSensorData(kTrajectoryId, kSensorId[2],
                         common::make_unique<Data>(third));

/*

h:{0,100,400}
v:{0,200,500}
i:{0,300}
o:{0,600}
*/
  ASSERT_EQ(7, received.size()); //{0,0,0,0,100,200,300}
  EXPECT_EQ(100, common::ToUniversal(received[4].second.time));
  EXPECT_EQ(kSensorId[0], received[4].first);
  EXPECT_EQ(200, common::ToUniversal(received[5].second.time));
  EXPECT_EQ(kSensorId[1], received[5].first);
  EXPECT_EQ(300, common::ToUniversal(received[6].second.time));
  EXPECT_EQ(kSensorId[2], received[6].first);

  collator.Flush();  //刷新

  ASSERT_EQ(10, received.size());         //10个数据,所有sensor采集的数据
  EXPECT_EQ(kSensorId[0], received[7].first);
  EXPECT_EQ(500, common::ToUniversal(received[8].second.time));
  EXPECT_EQ(kSensorId[1], received[8].first);
  EXPECT_EQ(600, common::ToUniversal(received[9].second.time));
  EXPECT_EQ(kSensorId[3], received[9].first);
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
