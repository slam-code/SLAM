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

#include "cartographer/mapping_2d/xy_index.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping_2d {
namespace {

TEST(XYIndexTest, CellLimitsToProto) {
  const CellLimits limits(1, 2);
  const auto proto = ToProto(limits);
  EXPECT_EQ(limits.num_x_cells, proto.num_x_cells());//序列化后相等
  EXPECT_EQ(limits.num_y_cells, proto.num_y_cells());
}

TEST(XYIndexTest, CellLimitsProtoConstructor) {
  proto::CellLimits limits;
  limits.set_num_x_cells(1);
  limits.set_num_y_cells(2);

  auto native = CellLimits(limits); //反序列化,native是c++的CellLimits对象
  EXPECT_EQ(limits.num_x_cells(), native.num_x_cells);
  EXPECT_EQ(limits.num_y_cells(), native.num_y_cells);
}

TEST(XYIndexTest, XYIndexRangeIterator) {
  const Eigen::Array2i min(1, 2);//x,y的下界
  const Eigen::Array2i max(3, 4);//x,y的上界
  XYIndexRangeIterator it(min, max);
  EXPECT_TRUE((min == *it.begin()).all()) << *it.begin();
  EXPECT_TRUE((Eigen::Array2i(1, 5) == *it.end()).all()) << *it.end();
  EXPECT_TRUE((min == *it).all()) << *it;
  int num_indices = 0;
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(min, max)) {
    LOG(INFO) << xy_index;//
    EXPECT_TRUE((xy_index >= min).all());
    EXPECT_TRUE((xy_index <= max).all());
    ++num_indices;
  }
  EXPECT_EQ(9, num_indices);// 3-1+1 ,* ,4-2+1  ==3*3共9个
}
/*
I0614 11:31:47.790160 20221 xy_index_test.cc:51] 1 2
I0614 11:31:47.790268 20221 xy_index_test.cc:51] 2 2
I0614 11:31:47.790282 20221 xy_index_test.cc:51] 3 2
I0614 11:31:47.790294 20221 xy_index_test.cc:51] 1 3
I0614 11:31:47.790318 20221 xy_index_test.cc:51] 2 3
I0614 11:31:47.790329 20221 xy_index_test.cc:51] 3 3
I0614 11:31:47.790341 20221 xy_index_test.cc:51] 1 4
I0614 11:31:47.790372 20221 xy_index_test.cc:51] 2 4
I0614 11:31:47.790382 20221 xy_index_test.cc:51] 3 4
*/
}  // namespace
}  // namespace mapping_2d
}  // namespace cartographer
