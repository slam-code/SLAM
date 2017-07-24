
#ifndef CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
#define CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace transform {

/*
插值变换,基于时间有序的变换,支持在队列中按时间顺序查找
*/
// A time-ordered buffer of transforms that supports interpolated lookups.
class TransformInterpolationBuffer {
 public:
 	//函数,返回智能指针
  static std::unique_ptr<TransformInterpolationBuffer> FromTrajectory(
      const mapping::proto::Trajectory& trajectory);

/*
当缓冲区不够时,删除队首
*/
  // Adds a new transform to the buffer and removes the oldest transform if the
  // buffer size limit is exceeded.
  void Push(common::Time time, const transform::Rigid3d& transform);

  // Returns true if an interpolated transfrom can be computed at 'time'.插值变换
  bool Has(common::Time time) const;

  // Returns an interpolated transform at 'time'. CHECK()s that a transform at
  // 'time' is available.
  transform::Rigid3d Lookup(common::Time time) const;

/*
返回缓冲区内变换的最早时间
*/
  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  common::Time earliest_time() const;
/*
最晚时间
*/
  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  common::Time latest_time() const;

  // Returns true if the buffer is empty.
  bool empty() const;

 private:
  struct TimestampedTransform {
    common::Time time;
    transform::Rigid3d transform;//变换矩阵
  };

  std::deque<TimestampedTransform> deque_;
};

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
