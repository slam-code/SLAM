
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:sensor/range_data.h


#ifndef CARTOGRAPHER_SENSOR_RANGE_DATA_H_
#define CARTOGRAPHER_SENSOR_RANGE_DATA_H_

#include "cartographer/common/port.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/sensor.pb.h"

namespace cartographer {
namespace sensor {

/*
RangeData:
数据成员包括
1),原始位置,{x0,y0,z0}
2),返回点云,{x,y,z}
3),缺失点云,标识free space.
*/
// Rays begin at 'origin'. 'returns' are the points where obstructions were
// detected. 'misses' are points in the direction of rays for which no return
// was detected, and were inserted at a configured distance. It is assumed that
// between the 'origin' and 'misses' is free space.
struct RangeData {
  Eigen::Vector3f origin;//{x0,y0,z0},sensor坐标。
  PointCloud returns;    //反射位置{x,y,z}，表征有物体反射。
  PointCloud misses;     //无反射,自由空间
};

// Converts 'range_data' to a proto::RangeData. 序列化
proto::RangeData ToProto(const RangeData& range_data);

// Converts 'proto' to a RangeData. 反序列化
RangeData FromProto(const proto::RangeData& proto);

//对数据进行3d变换，转换为机器坐标
RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform);

//根据min_z和max_z把不在z轴范围内的点云丢弃，剪裁到给定范围
// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
RangeData CropRangeData(const RangeData& range_data, float min_z, float max_z);

//压缩后的点云数据
// Like RangeData but with compressed point clouds. The point order changes
// when converting from RangeData.
struct CompressedRangeData {
  Eigen::Vector3f origin;
  CompressedPointCloud returns;
  CompressedPointCloud misses;
};

CompressedRangeData Compress(const RangeData& range_data);

RangeData Decompress(const CompressedRangeData& compressed_range_Data);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_RANGE_DATA_H_


```
.

```

文件:sensor/range_data.cc


#include "cartographer/sensor/range_data.h"

#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {
/*

message PointCloud {     
  repeated float x = 3 [packed = true];
  repeated float y = 4 [packed = true];
  repeated float z = 5 [packed = true];
}

message CompressedPointCloud { 
  optional int32 num_points = 1;
  repeated int32 point_data = 3 [packed = true];
}

message RangeData {             
  optional transform.proto.Vector3f origin = 1;
  optional PointCloud point_cloud = 2;
  optional PointCloud missing_echo_point_cloud = 3;
}

PointCloud;//vector，元素是3*1f

*/
proto::RangeData ToProto(const RangeData& range_data) {
  proto::RangeData proto;
  *proto.mutable_origin() = transform::ToProto(range_data.origin);
  *proto.mutable_point_cloud() = ToProto(range_data.returns);
  *proto.mutable_missing_echo_point_cloud() = ToProto(range_data.misses);
  return proto;
}

RangeData FromProto(const proto::RangeData& proto) {
  auto range_data = RangeData{
      transform::ToEigen(proto.origin()), ToPointCloud(proto.point_cloud()),
      ToPointCloud(proto.missing_echo_point_cloud()),
  };
  return range_data;
}

/*

将sensor坐标变换为机器坐标。
*/
RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform) {
  return RangeData{
      transform * range_data.origin,
      TransformPointCloud(range_data.returns, transform),
      TransformPointCloud(range_data.misses, transform),
  };
}

/*
不在给定的z轴范围内的点云删除。
*/
RangeData CropRangeData(const RangeData& range_data, const float min_z,
                        const float max_z) {
  return RangeData{range_data.origin, Crop(range_data.returns, min_z, max_z),
                   Crop(range_data.misses, min_z, max_z)};
}

/*
压缩，有精度丢失。
*/
CompressedRangeData Compress(const RangeData& range_data) {
  return CompressedRangeData{
      range_data.origin, CompressedPointCloud(range_data.returns),
      CompressedPointCloud(range_data.misses),
  };
}
/*
解压缩，有精度丢失
*/
RangeData Decompress(const CompressedRangeData& compressed_range_data) {
  return RangeData{compressed_range_data.origin,
                   compressed_range_data.returns.Decompress(),
                   compressed_range_data.misses.Decompress()};
}

}  // namespace sensor
}  // namespace cartographer


```

```c++
测试代码:sensor/range_data_test.h


#include "cartographer/sensor/range_data.h"

#include <utility>
#include <vector>

#include "gmock/gmock.h"

namespace cartographer {
namespace sensor {
namespace {

using ::testing::Contains;
using ::testing::PrintToString;

// Custom matcher for Eigen::Vector3f entries.
MATCHER_P(ApproximatelyEquals, expected,
          string("is equal to ") + PrintToString(expected)) {
  return (arg - expected).isZero(0.001f);
}

TEST(RangeDataTest, Compression) {
  const std::vector<Eigen::Vector3f> returns = {Eigen::Vector3f(0, 1, 2),
                                                Eigen::Vector3f(4, 5, 6),
                                                Eigen::Vector3f(0, 1, 2)};
   //构造一个测量数据
  const RangeData range_data = {
      Eigen::Vector3f(1, 1, 1), returns, {Eigen::Vector3f(7, 8, 9)}};
  //压缩再解压，减少空间占用。
  const RangeData actual = Decompress(Compress(range_data));

//isApprox():true if *this is approximately equal to other, within the precision determined by prec.
 //压缩前后，精度比较
  EXPECT_TRUE(actual.origin.isApprox(Eigen::Vector3f(1, 1, 1), 1e-6));


  EXPECT_EQ(3, actual.returns.size());

  EXPECT_EQ(1, actual.misses.size());

  EXPECT_TRUE(actual.misses[0].isApprox(Eigen::Vector3f(7, 8, 9), 0.001f));

  // Returns will be reordered, so we compare in an unordered manner.
  EXPECT_EQ(3, actual.returns.size());

  //是否包含给定值，returns是乱序的。所以使用Contains()函数。
  EXPECT_THAT(actual.returns,
              Contains(ApproximatelyEquals(Eigen::Vector3f(0, 1, 2))));
  EXPECT_THAT(actual.returns,
              Contains(ApproximatelyEquals(Eigen::Vector3f(4, 5, 6))));
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
