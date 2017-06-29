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

#include <string>
#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

namespace cartographer_ros {
namespace {

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions() {
  auto file_resolver = cartographer::common::make_unique<
      cartographer::common::ConfigurationFileResolver>(
      std::vector<string>{FLAGS_configuration_directory});              //cartographer的配置文件夹:configuration_files
  const string code =
      file_resolver->GetFileContentOrDie(FLAGS_configuration_basename); //car_ros下的revo_lds.lua,根据.lua文件配置cartographer的options
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

//CreateNodeOptions:node_options.h
  return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),  //根据.lua文件配置options
                         CreateTrajectoryOptions(&lua_parameter_dictionary));
}

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 1e6;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) = LoadOptions();//通过std::tie解包tuple后，值会自动赋值给2个变量。

  Node node(node_options, &tf_buffer);

  //node.h
  node.StartTrajectoryWithDefaultTopics(trajectory_options);

  ::ros::spin();

  node.FinishAllTrajectories();
}

}  // namespace
}  // namespace cartographer_ros
/*

google_binary(cartographer_node
  SRCS
    node_main.cc
)


*/
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

//-configuration_directory $(find cartographer_ros)/configuration_files
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";

//-configuration_basename revo_lds.lua
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node");// //roslaunch cartographer_ros demo_revo_lds.launch,调用的主函数
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::ros::shutdown();
}
