
#ifndef CARTOGRAPHER_IO_FIXED_RATIO_SAMPLING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_FIXED_RATIO_SAMPLING_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"

namespace cartographer {
namespace io {

/*
FixedRatioSamplingPointsProcessor是采样滤波器,只让具有固定采样的点保留,其余全部 remove
*/
// Only let a fixed 'sampling_ratio' of points through. A 'sampling_ratio' of 1.
// makes this filter a no-op.
class FixedRatioSamplingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "fixed_ratio_sampler";

  FixedRatioSamplingPointsProcessor(double sampling_ratio,
                                    PointsProcessor* next);

  static std::unique_ptr<FixedRatioSamplingPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~FixedRatioSamplingPointsProcessor() override{};

  FixedRatioSamplingPointsProcessor(const FixedRatioSamplingPointsProcessor&) =
      delete;
  FixedRatioSamplingPointsProcessor& operator=(
      const FixedRatioSamplingPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const double sampling_ratio_;
  PointsProcessor* const next_;
  std::unique_ptr<common::FixedRatioSampler> sampler_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_FIXED_RATIO_SAMPLING_POINTS_PROCESSOR_H_
