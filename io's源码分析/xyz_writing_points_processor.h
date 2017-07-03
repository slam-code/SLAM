
#ifndef CARTOGRAPHER_IO_XYZ_WRITING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_XYZ_WRITING_POINTS_PROCESSOR_H_

#include <fstream>
#include <memory>
#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {
/*
XyzWriterPointsProcessor将points以{x,y,z}形式写入 FileWriter

*/
// Writes ASCII xyz points.
class XyzWriterPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName = "write_xyz";

  XyzWriterPointsProcessor(std::unique_ptr<FileWriter>, PointsProcessor* next);

  static std::unique_ptr<XyzWriterPointsProcessor> FromDictionary(
      FileWriterFactory file_writer_factory,
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~XyzWriterPointsProcessor() override {}

  XyzWriterPointsProcessor(const XyzWriterPointsProcessor&) = delete;
  XyzWriterPointsProcessor& operator=(const XyzWriterPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  PointsProcessor* const next_;
  std::unique_ptr<FileWriter> file_writer_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_XYZ_WRITING_POINTS_PROCESSOR_H_
