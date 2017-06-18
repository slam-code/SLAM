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

#include "cartographer/mapping_2d/ray_casting.h"

namespace cartographer {
namespace mapping_2d {

namespace {

// Factor for subpixel accuracy of start and end point.
constexpr int kSubpixelScale = 1000;

/*
CastRay函数的功能:
对给定的{x0,y0},{x1,y1}划定的矩形框,都用visitor访问一遍

*/
// We divide each pixel in kSubpixelScale x kSubpixelScale subpixels. 'begin'
// and 'end' are coordinates at subpixel precision. We compute all pixels in
// which some part of the line segment connecting 'begin' and 'end' lies.
void CastRay(const Eigen::Array2i& begin, const Eigen::Array2i& end,
             const std::function<void(const Eigen::Array2i&)>& visitor) {
  // For simplicity, we order 'begin' and 'end' by their x coordinate.
  if (begin.x() > end.x()) {
    CastRay(end, begin, visitor);
    return;
  }

  CHECK_GE(begin.x(), 0);
  CHECK_GE(begin.y(), 0);
  CHECK_GE(end.y(), 0);

  // Special case: We have to draw a vertical line in full pixels, as 'begin'
  // and 'end' have the same full pixel x coordinate.
  if (begin.x() / kSubpixelScale == end.x() / kSubpixelScale) { // begin==end
    Eigen::Array2i current(begin.x() / kSubpixelScale,
                           std::min(begin.y(), end.y()) / kSubpixelScale);
    const int end_y = std::max(begin.y(), end.y()) / kSubpixelScale;
    for (; current.y() <= end_y; ++current.y()) {
      visitor(current);
    }
    return;
  }

  const int64 dx = end.x() - begin.x();                       //水平方向dx
  const int64 dy = end.y() - begin.y();                       //垂直方向dy
  const int64 denominator = 2 * kSubpixelScale * dx;          //分母

  // The current full pixel coordinates. We begin at 'begin'.
  Eigen::Array2i current = begin / kSubpixelScale;

  // To represent subpixel centers, we use a factor of 2 * 'kSubpixelScale' in
  // the denominator.
  // +-+-+-+ -- 1 = (2 * kSubpixelScale) / (2 * kSubpixelScale)       //就是 2k/2k   第k个子像素的上边界        
  // | | | |
  // +-+-+-+
  // | | | |
  // +-+-+-+ -- top edge of first subpixel = 2 / (2 * kSubpixelScale) //就是 2/(2*k) 第一个子像素的上边界  y0+1
  // | | | | -- center of first subpixel = 1 / (2 * kSubpixelScale)   //就是 1/(2k)  第一个子像素的中心   y0+1/2
  // +-+-+-+ -- 0 = 0 / (2 * kSubpixelScale)                          //就是 0/(2k)  第一个子像素的下边界  y0

  // The center of the subpixel part of 'begin.y()' assuming the
  // 'denominator', i.e., sub_y / denominator is in (0, 1).
  int64 sub_y = (2 * (begin.y() % kSubpixelScale) + 1) * dx;          //第一个子像素的y

  // The distance from the from 'begin' to the right pixel border, to be divided
  // by 2 * 'kSubpixelScale'.
  const int first_pixel =                                             //第一个子像素
      2 * kSubpixelScale - 2 * (begin.x() % kSubpixelScale) - 1;
  // The same from the left pixel border to 'end'.
  const int last_pixel = 2 * (end.x() % kSubpixelScale) + 1;          //最后一个子像素

  // The full pixel x coordinate of 'end'.
  const int end_x = std::max(begin.x(), end.x()) / kSubpixelScale;    //最后一个子像素的x

  // Move from 'begin' to the next pixel border to the right.
  sub_y += dy * first_pixel;                                          //所有y方向上的增加delta,以确保所有的子像素都能被visitor
  if (dy > 0) {
    while (true) {
      visitor(current);
      while (sub_y > denominator) {
        sub_y -= denominator;
        ++current.y();
        visitor(current);
      }
      ++current.x();
      if (sub_y == denominator) {
        sub_y -= denominator;
        ++current.y();
      }
      if (current.x() == end_x) {
        break;
      }
      // Move from one pixel border to the next.
      sub_y += dy * 2 * kSubpixelScale;
    }
    // Move from the pixel border on the right to 'end'.
    sub_y += dy * last_pixel;
    visitor(current);
    while (sub_y > denominator) {
      sub_y -= denominator;
      ++current.y();
      visitor(current);
    }
    CHECK_NE(sub_y, denominator);
    CHECK_EQ(current.y(), end.y() / kSubpixelScale);
    return;
  }

  // Same for lines non-ascending in y coordinates.
  while (true) {
    visitor(current);
    while (sub_y < 0) {
      sub_y += denominator;
      --current.y();
      visitor(current);
    }
    ++current.x();
    if (sub_y == 0) {
      sub_y += denominator;
      --current.y();
    }
    if (current.x() == end_x) {
      break;
    }
    sub_y += dy * 2 * kSubpixelScale;
  }
  sub_y += dy * last_pixel;
  visitor(current);
  while (sub_y < 0) {
    sub_y += denominator;
    --current.y();
    visitor(current);
  }
  CHECK_NE(sub_y, 0);
  CHECK_EQ(current.y(), end.y() / kSubpixelScale);
}

}  // namespace

/*

*/
void CastRays(const sensor::RangeData& range_data, const MapLimits& limits,
              const std::function<void(const Eigen::Array2i&)>& hit_visitor,
              const std::function<void(const Eigen::Array2i&)>& miss_visitor) {
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  const Eigen::Array2i begin =
      superscaled_limits.GetXYIndexOfCellContainingPoint(range_data.origin.x(),
                                                         range_data.origin.y());

  // Compute and add the end points.
  std::vector<Eigen::Array2i> ends;
  ends.reserve(range_data.returns.size());
  for (const Eigen::Vector3f& hit : range_data.returns) { //对每一个hit的3f点
    ends.push_back(
        superscaled_limits.GetXYIndexOfCellContainingPoint(hit.x(), hit.y()));
    hit_visitor(ends.back() / kSubpixelScale);
  }

  // Now add the misses.
  for (const Eigen::Array2i& end : ends) {              //miss的点
    CastRay(begin, end, miss_visitor);
  }

  // Finally, compute and add empty rays based on misses in the scan.
  for (const Eigen::Vector3f& missing_echo : range_data.misses) { //没有反射的点
    CastRay(begin,
            superscaled_limits.GetXYIndexOfCellContainingPoint(
                missing_echo.x(), missing_echo.y()),
            miss_visitor);
  }
}

}  // namespace mapping_2d
}  // namespace cartographer
