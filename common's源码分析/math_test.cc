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

#include "cartographer/common/math.h"

#include "gtest/gtest.h"

namespace cartographer {
namespace common {
namespace {

TEST(MathTest, testPower) {
  EXPECT_EQ(0., Power(0, 42));// 0的42次方==0
  EXPECT_EQ(1., Power(0, 0)); //0^0 ==0
  EXPECT_EQ(1., Power(1, 0)); // 1^0 ==0
  EXPECT_EQ(1., Power(1, 42));//1^42==1
  EXPECT_EQ(4., Power(2, 2)); //2^2==4
}

TEST(MathTest, testPow2) {
  EXPECT_EQ(0., Pow2(0)); //0^2==0
  EXPECT_EQ(1., Pow2(1)); //1^2==1
  EXPECT_EQ(4., Pow2(2)); //2^2==4
  EXPECT_EQ(49., Pow2(7));//7^2==49
}

TEST(MathTest, testDeg2rad) {
  EXPECT_NEAR(M_PI, DegToRad(180.), 1e-9);				// 180° ==pi
  EXPECT_NEAR(2. * M_PI, DegToRad(360. - 1e-9), 1e-6);//360° ==2pi
}

TEST(MathTest, testRad2deg) {
  EXPECT_NEAR(180., RadToDeg(M_PI), 1e-9);			   //pi ==180°
  EXPECT_NEAR(360., RadToDeg(2. * M_PI - 1e-9), 1e-6);//2pi ==360°
}

TEST(MathTest, testNormalizeAngleDifference) {
  EXPECT_NEAR(0., NormalizeAngleDifference(0.), 1e-9);     		//0==0
  EXPECT_NEAR(M_PI, NormalizeAngleDifference(M_PI), 1e-9); 		//pi==oi
  EXPECT_NEAR(-M_PI, NormalizeAngleDifference(-M_PI), 1e-9);	//-pi==-pi
  EXPECT_NEAR(0., NormalizeAngleDifference(2. * M_PI), 1e-9);   //2pi==0
  EXPECT_NEAR(M_PI, NormalizeAngleDifference(5. * M_PI), 1e-9); //5pi==pi
  EXPECT_NEAR(-M_PI, NormalizeAngleDifference(-5. * M_PI), 1e-9);//-5pi=-pi
}

}  // namespace
}  // namespace common
}  // namespace cartographer
