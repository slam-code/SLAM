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

#include <fstream>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"
/*
点云是在和目标表面特性的海量点集合。
根据激光测量原理得到的点云，包括三维坐标（XYZ）和激光反射强度（Intensity）。
根据摄影测量原理得到的点云，包括三维坐标（XYZ）和颜色信息（RGB）。
结合激光测量和摄影测量原理得到点云，包括三维坐标（XYZ）、激光反射强度（Intensity）和颜色信息（RGB）。
在获取物体表面每个采样点的空间坐标后，得到的是一个点的集合，称之为“点云”(Point Cloud)。

pcd文件存储了每个点的x,y,z坐标以及r,g,b,a颜色信息

*/
namespace cartographer {
namespace io {

/*
PcdWritingPointsProcessor类将点云按照pcd格式存储在pcd文件中.

1,构造函数初始化一个文件类名,
2,成员函数FromDictionary 从配置文件读取 文件名
3,Process()和Flush()负责写入文件

4,数据成员next_指向 点云处理类
*/
// Streams a PCD file to disk. The header is written in 'Flush'.
class PcdWritingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName = "write_pcd";
  PcdWritingPointsProcessor(std::unique_ptr<FileWriter> file_writer,
                            PointsProcessor* next);

  static std::unique_ptr<PcdWritingPointsProcessor> FromDictionary(
      FileWriterFactory file_writer_factory,
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~PcdWritingPointsProcessor() override {}

  PcdWritingPointsProcessor(const PcdWritingPointsProcessor&) = delete;
  PcdWritingPointsProcessor& operator=(const PcdWritingPointsProcessor&) =
      delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  PointsProcessor* const next_;

  int64 num_points_;
  bool has_colors_;
  std::unique_ptr<FileWriter> file_writer_;
};

}  // namespace io
}  // namespace cartographer
