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

#include "cartographer/sensor/ordered_multi_queue.h"

#include <vector>

#include "cartographer/common/make_unique.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace sensor {
namespace {

class OrderedMultiQueueTest : public ::testing::Test {
 protected:
  // These are keys are chosen so that they sort first, second, third.
  const QueueKey kFirst{1, "a"};
  const QueueKey kSecond{1, "b"};
  const QueueKey kThird{2, "b"};

//void AddQueue(const QueueKey& queue_key, Callback callback);
  void SetUp() {//初始化函数,定义queue_key的比较函数(以后用于比较):检查values_的最大值是否小于欲添加的值.
    for (const auto& queue_key : {kFirst, kSecond, kThird}) {
      queue_.AddQueue(queue_key, [this](std::unique_ptr<Data> data) {
        if (!values_.empty()) {
          EXPECT_GE(data->time, values_.back().time);
        }
        values_.push_back(*data);
      });
    }
  }

  std::unique_ptr<Data> MakeImu(const int ordinal) {//时间,序列
    return common::make_unique<Data>(
        common::FromUniversal(ordinal),
        Data::Imu{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});
  }

  std::vector<Data> values_;
  OrderedMultiQueue queue_;
};

/*测试夹具（Test Fixtures）
TEST_F提供了一个初始化函数（SetUp）和一个清理函数(TearDown)，
在TEST_F中使用的变量可以在初始化函数SetUp中初始化，
在TearDown中销毁，并且所有的TEST_F是互相独立的，都是在初始化以后的状态开始运行，
一个TEST_F不会影响另一个TEST_F所使用的数据.
用TEST_F定义测试，写法与TEST相同，但测试用例名必须为上面定义的类名。
*/
TEST_F(OrderedMultiQueueTest, Ordering) {
  // void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);
  queue_.Add(kFirst, MakeImu(0));//
  queue_.Add(kFirst, MakeImu(4));
  queue_.Add(kFirst, MakeImu(5));
  queue_.Add(kFirst, MakeImu(6));
  EXPECT_TRUE(values_.empty());  //4, 为空?


  queue_.Add(kSecond, MakeImu(0));
  queue_.Add(kSecond, MakeImu(1));
  EXPECT_TRUE(values_.empty());


  queue_.Add(kThird, MakeImu(0));
  queue_.Add(kThird, MakeImu(2));
  EXPECT_EQ(values_.size(), 4);

  queue_.Add(kSecond, MakeImu(3));
  EXPECT_EQ(values_.size(), 5);

  queue_.Add(kSecond, MakeImu(7));
  queue_.Add(kThird, MakeImu(8));
  queue_.Flush();//应该调用,否则出错

  EXPECT_EQ(11, values_.size());// f:4,s:4,t:3
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
  EXPECT_TRUE(values_.empty());
  queue_.MarkQueueAsFinished(kSecond);
  EXPECT_TRUE(values_.empty());
  queue_.MarkQueueAsFinished(kThird);

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
  queue_.MarkQueueAsFinished(kFirst);
  EXPECT_EQ(values_.size(), 2);
  queue_.MarkQueueAsFinished(kSecond);
  EXPECT_EQ(values_.size(), 4);
  queue_.MarkQueueAsFinished(kThird);
  EXPECT_EQ(values_.size(), 4);
}

}  // namespace
}  // namespace sensor
}  // namespace cartographer
