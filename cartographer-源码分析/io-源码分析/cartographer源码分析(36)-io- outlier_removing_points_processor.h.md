
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:coloring_points_processor.h


#ifndef CARTOGRAPHER_IO_OUTLIER_REMOVING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_OUTLIER_REMOVING_POINTS_PROCESSOR_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping_3d/hybrid_grid.h"

namespace cartographer {
namespace io {

/*
OutlierRemovingPointsProcessor是PointsProcessor的第八个子类(8).
OutlierRemovingPointsProcessor：
作用：; 异常体素过滤器,Remove有移动痕迹的体素。只保留没有移动的体素

数据成员：

const double voxel_size_;
PointsProcessor* const next_;
State state_;
mapping_3d::HybridGridBase<VoxelData> voxels_;
*/
// Voxel filters the data and only passes on points that we believe are on
// non-moving objects.
class OutlierRemovingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "voxel_filter_and_remove_moving_objects";

  OutlierRemovingPointsProcessor(double voxel_size, PointsProcessor* next);

  static std::unique_ptr<OutlierRemovingPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~OutlierRemovingPointsProcessor() override {}

  OutlierRemovingPointsProcessor(const OutlierRemovingPointsProcessor&) =
      delete;
  OutlierRemovingPointsProcessor& operator=(
      const OutlierRemovingPointsProcessor&) = delete;
//删除移动的体素。
  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  // To reduce memory consumption by not having to keep all rays in memory, we
  // filter outliers in three phases each going over all data: First we compute
  // all voxels containing any hits, then we compute the rays passing through
  // each of these voxels, and finally we output all hits in voxels that are
  // considered obstructed.
  struct VoxelData {
    int hits = 0;
    int rays = 0;
  };
  enum class State {
    kPhase1,
    kPhase2,
    kPhase3,
  };

  // First phase counts the number of hits per voxel.
  void ProcessInPhaseOne(const PointsBatch& batch);

  // Second phase counts how many rays pass through each voxel. This is only
  // done for voxels that contain hits. This is to reduce memory consumption by
  // not adding data to free voxels.
  void ProcessInPhaseTwo(const PointsBatch& batch);

  // Third phase produces the output containing all inliers. We consider each
  // hit an inlier if it is inside a voxel that has a sufficiently high
  // hit-to-ray ratio.
  void ProcessInPhaseThree(std::unique_ptr<PointsBatch> batch);

  const double voxel_size_; //体素大小
  PointsProcessor* const next_;
  State state_;
  mapping_3d::HybridGridBase<VoxelData> voxels_; //包含多个体素的网格Grid。
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_OUTLIER_REMOVING_POINTS_PROCESSOR_H_



```


```c++
coloring_points_processor.cc




#include "cartographer/io/outlier_removing_points_processor.h"

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "glog/logging.h"

namespace cartographer {
namespace io {
/*
VOXEL_SIZE = 5e-2
*/
std::unique_ptr<OutlierRemovingPointsProcessor>
OutlierRemovingPointsProcessor::FromDictionary(
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return common::make_unique<OutlierRemovingPointsProcessor>(
      dictionary->GetDouble("voxel_size"), next); //构造一个对象,返回一个智能指针
}

/*
构造函数,传递一个 voxel_size 和  PointsProcessor* next
*/
OutlierRemovingPointsProcessor::OutlierRemovingPointsProcessor(
    const double voxel_size, PointsProcessor* next)
    : voxel_size_(voxel_size),
      next_(next),
      state_(State::kPhase1),
      voxels_(voxel_size_) {
  LOG(INFO) << "Marking hits...";
}

/*
根据3个不同的state分别处理 points
*/
void OutlierRemovingPointsProcessor::Process(
    std::unique_ptr<PointsBatch> points) {
  switch (state_) {
    case State::kPhase1:
      ProcessInPhaseOne(*points);
      break;

    case State::kPhase2:
      ProcessInPhaseTwo(*points);
      break;

    case State::kPhase3:
      ProcessInPhaseThree(std::move(points));
      break;
  }
}

/*
更新state,并返回 FlushResult结果
*/
PointsProcessor::FlushResult OutlierRemovingPointsProcessor::Flush() {
  switch (state_) {
    case State::kPhase1:
      LOG(INFO) << "Counting rays...";
      state_ = State::kPhase2;
      return FlushResult::kRestartStream;

    case State::kPhase2:
      LOG(INFO) << "Filtering outliers...";
      state_ = State::kPhase3;
      return FlushResult::kRestartStream;

    case State::kPhase3:
      CHECK(next_->Flush() == FlushResult::kFinished)
          << "Voxel filtering and outlier removal must be configured to occur "
             "after any stages that require multiple passes.";
             // multiple passes：多次传输。
      return FlushResult::kFinished;
  }
  LOG(FATAL);
}


/*
state 1，统计光线的数量。
*/
void OutlierRemovingPointsProcessor::ProcessInPhaseOne(
    const PointsBatch& batch) {
  for (size_t i = 0; i < batch.points.size(); ++i) {
    ++voxels_.mutable_value(voxels_.GetCellIndex(batch.points[i]))->hits;
  }
}

/*


*/
void OutlierRemovingPointsProcessor::ProcessInPhaseTwo(
    const PointsBatch& batch) {
  // TODO(whess): This samples every 'voxel_size' distance and could be improved
  // by better ray casting, and also by marking the hits of the current range
  // data to be excluded.
  for (size_t i = 0; i < batch.points.size(); ++i) {
    const Eigen::Vector3f delta = batch.points[i] - batch.origin;
    const float length = delta.norm();
    for (float x = 0; x < length; x += voxel_size_) {
      const auto index =
          voxels_.GetCellIndex(batch.origin + (x / length) * delta);
      if (voxels_.value(index).hits > 0) {
        ++voxels_.mutable_value(index)->rays;
      }
    }
  }
}

void OutlierRemovingPointsProcessor::ProcessInPhaseThree(
    std::unique_ptr<PointsBatch> batch) {
  constexpr double kMissPerHitLimit = 3;
  std::vector<int> to_remove;
  for (size_t i = 0; i < batch->points.size(); ++i) {
    const auto voxel = voxels_.value(voxels_.GetCellIndex(batch->points[i]));
    if (!(voxel.rays < kMissPerHitLimit * voxel.hits)) {
      to_remove.push_back(i);
    }
  }
  RemovePoints(to_remove, batch.get());
  next_->Process(std::move(batch));
}

}  // namespace io
}  // namespace cartographer


```

本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode
