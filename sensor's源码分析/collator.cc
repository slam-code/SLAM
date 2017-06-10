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
  for (const auto& sensor_id : expected_sensor_ids) { //对于每一个轨迹线+传感器,设置一个key
    const auto queue_key = QueueKey{trajectory_id, sensor_id};

    //添加一个名为key的队列,并设置回调函数
    queue_.AddQueue(queue_key,
                    [callback, sensor_id](std::unique_ptr<Data> data) {
                      callback(sensor_id, std::move(data));
                    });

    //map<int,vector<key>>:添加轨迹线对应的key
    queue_keys_[trajectory_id].push_back(queue_key);
  }
}

void Collator::FinishTrajectory(const int trajectory_id) {
  for (const auto& queue_key : queue_keys_[trajectory_id]) {
    queue_.MarkQueueAsFinished(queue_key);
  }
}

void Collator::AddSensorData(const int trajectory_id, const string& sensor_id,
                             std::unique_ptr<Data> data) {
  queue_.Add(QueueKey{trajectory_id, sensor_id}, std::move(data));
}

void Collator::Flush() { queue_.Flush(); }

int Collator::GetBlockingTrajectoryId() const {
  return queue_.GetBlocker().trajectory_id;
}

}  // namespace sensor
}  // namespace cartographer
