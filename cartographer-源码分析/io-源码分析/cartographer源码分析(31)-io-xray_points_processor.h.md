
源码可在https://github.com/learnmoreonce/SLAM 下载


``` c++
 
文件:xray_points_processor.h

#ifndef CARTOGRAPHER_IO_XRAY_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_XRAY_POINTS_PROCESSOR_H_

#include <map>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping/detect_floors.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/transform/rigid_transform.h"

/*配置文件概览：
VOXEL_SIZE = 5e-2

YZ_TRANSFORM =  {
  translation = { 0., 0., 0. },
  rotation = { 0. , 0., math.pi, },
}

{
    action = "write_xray_image",
    voxel_size = VOXEL_SIZE,
    filename = "xray_yz_all_intensity",
    transform = YZ_TRANSFORM,
},

*/

namespace cartographer {
namespace io {
/*
xray_points_processor.h是PointsProcessor的第三个子类(3).
功能:将x射线以体素为voxel_size写入image。

数据成员:
1), 
2), 

*/
// Creates X-ray cuts through the points with pixels being 'voxel_size' big.
class XRayPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "write_xray_image";
  XRayPointsProcessor(double voxel_size, const transform::Rigid3f& transform,
                      const std::vector<mapping::Floor>& floors,
                      const string& output_filename,
                      FileWriterFactory file_writer_factory,
                      PointsProcessor* next);

  static std::unique_ptr<XRayPointsProcessor> FromDictionary(
      const mapping::proto::Trajectory& trajectory,
      FileWriterFactory file_writer_factory,
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~XRayPointsProcessor() override {}

//将点云写入.png文件，然后流水到下一处理器
  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

  Eigen::AlignedBox3i bounding_box() const { return bounding_box_; }

 private:
  struct ColumnData {
    double sum_r = 0.;
    double sum_g = 0.;
    double sum_b = 0.;
    uint32_t count = 0;
  };

  struct Aggregation {
    mapping_3d::HybridGridBase<bool> voxels;
    std::map<std::pair<int, int>, ColumnData> column_data;
  };

  void WriteVoxels(const Aggregation& aggregation,
                   FileWriter* const file_writer);
  void Insert(const PointsBatch& batch, const transform::Rigid3f& transform,
              Aggregation* aggregation);

  PointsProcessor* const next_;
  FileWriterFactory file_writer_factory_; //负责文件写入的工厂

  // If empty, we do not separate into floors.
  std::vector<mapping::Floor> floors_; //楼层

  const string output_filename_;       //文件名称
  const transform::Rigid3f transform_; //点云涉及的变换

  // Only has one entry if we do not separate into floors.
  std::vector<Aggregation> aggregations_; //聚集数据

  // Bounding box containing all cells with data in all 'aggregations_'.
  Eigen::AlignedBox3i bounding_box_; //边界框
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_XRAY_POINTS_PROCESSOR_H_


```


```c++
xray_points_processor.cc



#include "cartographer/io/xray_points_processor.h"

#include <cmath>
#include <string>

#include "Eigen/Core"
#include "cairo/cairo.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/math.h"
#include "cartographer/io/cairo_types.h"
#include "cartographer/mapping/detect_floors.h"
#include "cartographer/mapping_3d/hybrid_grid.h"

namespace cartographer {
namespace io {
namespace {

struct PixelData {
  size_t num_occupied_cells_in_column = 0;
  double mean_r = 0.;
  double mean_g = 0.;
  double mean_b = 0.;
};

using PixelDataMatrix =
    Eigen::Matrix<PixelData, Eigen::Dynamic, Eigen::Dynamic>;

double Mix(const double a, const double b, const double t) {
  return a * (1. - t) + t * b;
}

//cairo回调函数，将data写入FileWriter
cairo_status_t CairoWriteCallback(void* const closure,
                                  const unsigned char* data,
                                  const unsigned int length) {
  if (static_cast<FileWriter*>(closure)->Write(
          reinterpret_cast<const char*>(data), length)) {
    return CAIRO_STATUS_SUCCESS;
  }
  return CAIRO_STATUS_WRITE_ERROR;
}

/*
使用cairo库将mat以png文件写入filename
*/
// Write 'mat' as a pleasing-to-look-at PNG into 'filename'
void WritePng(const PixelDataMatrix& mat, FileWriter* const file_writer) {
  const int stride =
      cairo_format_stride_for_width(CAIRO_FORMAT_ARGB32, mat.cols());
  CHECK_EQ(stride % 4, 0);
  std::vector<uint32_t> pixels(stride / 4 * mat.rows(), 0.);

  float max = std::numeric_limits<float>::min();
  for (int y = 0; y < mat.rows(); ++y) {
    for (int x = 0; x < mat.cols(); ++x) {
      const PixelData& cell = mat(y, x);
      if (cell.num_occupied_cells_in_column == 0.) {
        continue;
      }
      max = std::max<float>(max, std::log(cell.num_occupied_cells_in_column));
    }
  }

  for (int y = 0; y < mat.rows(); ++y) {
    for (int x = 0; x < mat.cols(); ++x) {
      const PixelData& cell = mat(y, x);
      if (cell.num_occupied_cells_in_column == 0.) {
        pixels[y * stride / 4 + x] =
            (255 << 24) | (255 << 16) | (255 << 8) | 255;
        continue;
      }

      // We use a logarithmic weighting for how saturated a pixel will be. The
      // basic idea here was that walls (full height) are fully saturated, but
      // details like chairs and tables are still well visible.
      const float saturation =
          std::log(cell.num_occupied_cells_in_column) / max;
      double mean_r_in_column = (cell.mean_r / 255.);
      double mean_g_in_column = (cell.mean_g / 255.);
      double mean_b_in_column = (cell.mean_b / 255.);

      double mix_r = Mix(1., mean_r_in_column, saturation);
      double mix_g = Mix(1., mean_g_in_column, saturation);
      double mix_b = Mix(1., mean_b_in_column, saturation);

      const int r = common::RoundToInt(mix_r * 255.);
      const int g = common::RoundToInt(mix_g * 255.);
      const int b = common::RoundToInt(mix_b * 255.);
      pixels[y * stride / 4 + x] = (255 << 24) | (r << 16) | (g << 8) | b;
    }
  }

  // TODO(hrapp): cairo_image_surface_create_for_data does not take ownership of
  // the data until the surface is finalized. Once it is finalized though,
  // cairo_surface_write_to_png fails, complaining that the surface is already
  // finalized. This makes it pretty hard to pass back ownership of the image to
  // the caller.
  cairo::UniqueSurfacePtr surface(
      cairo_image_surface_create_for_data(
          reinterpret_cast<unsigned char*>(pixels.data()), CAIRO_FORMAT_ARGB32,
          mat.cols(), mat.rows(), stride),
      cairo_surface_destroy);
  CHECK_EQ(cairo_surface_status(surface.get()), CAIRO_STATUS_SUCCESS);
  CHECK_EQ(cairo_surface_write_to_png_stream(surface.get(), &CairoWriteCallback,
                                             file_writer),
           CAIRO_STATUS_SUCCESS);
  CHECK(file_writer->Close());
}

bool ContainedIn(const common::Time& time,
                 const std::vector<mapping::Timespan>& timespans) {
  for (const mapping::Timespan& timespan : timespans) {
    if (timespan.start <= time && time <= timespan.end) {
      return true;
    }
  }
  return false;
}

}  // namespace

XRayPointsProcessor::XRayPointsProcessor(
    const double voxel_size, const transform::Rigid3f& transform,
    const std::vector<mapping::Floor>& floors, const string& output_filename,
    FileWriterFactory file_writer_factory, PointsProcessor* const next)
    : next_(next),
      file_writer_factory_(file_writer_factory),
      floors_(floors),
      output_filename_(output_filename),
      transform_(transform) {
  for (size_t i = 0; i < (floors_.empty() ? 1 : floors.size()); ++i) {
    aggregations_.emplace_back(
        Aggregation{mapping_3d::HybridGridBase<bool>(voxel_size), {}});
  }
}

std::unique_ptr<XRayPointsProcessor> XRayPointsProcessor::FromDictionary(
    const mapping::proto::Trajectory& trajectory,
    FileWriterFactory file_writer_factory,
    common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  std::vector<mapping::Floor> floors;
  if (dictionary->HasKey("separate_floors") &&
      dictionary->GetBool("separate_floors")) {
    floors = mapping::DetectFloors(trajectory);
  }

  return common::make_unique<XRayPointsProcessor>(
      dictionary->GetDouble("voxel_size"),
      transform::FromDictionary(dictionary->GetDictionary("transform").get())
          .cast<float>(),
      floors, dictionary->GetString("filename"), file_writer_factory, next);
}

void XRayPointsProcessor::WriteVoxels(const Aggregation& aggregation,
                                      FileWriter* const file_writer) {
  if (bounding_box_.isEmpty()) {
    LOG(WARNING) << "Not writing output: bounding box is empty.";
    return;
  }

  // Returns the (x, y) pixel of the given 'index'.
  const auto voxel_index_to_pixel = [this](const Eigen::Array3i& index) {
    // We flip the y axis, since matrices rows are counted from the top.
    return Eigen::Array2i(bounding_box_.max()[1] - index[1],
                          bounding_box_.max()[2] - index[2]);
  };

  // Hybrid grid uses X: forward, Y: left, Z: up.
  // For the screen we are using. X: right, Y: up
  const int xsize = bounding_box_.sizes()[1] + 1;
  const int ysize = bounding_box_.sizes()[2] + 1;
  PixelDataMatrix image = PixelDataMatrix(ysize, xsize);
  for (mapping_3d::HybridGridBase<bool>::Iterator it(aggregation.voxels);
       !it.Done(); it.Next()) {
    const Eigen::Array3i cell_index = it.GetCellIndex();
    const Eigen::Array2i pixel = voxel_index_to_pixel(cell_index);
    PixelData& pixel_data = image(pixel.y(), pixel.x());
    const auto& column_data = aggregation.column_data.at(
        std::make_pair(cell_index[1], cell_index[2]));
    pixel_data.mean_r = column_data.sum_r / column_data.count;
    pixel_data.mean_g = column_data.sum_g / column_data.count;
    pixel_data.mean_b = column_data.sum_b / column_data.count;
    ++pixel_data.num_occupied_cells_in_column;
  }
  WritePng(image, file_writer);
}

void XRayPointsProcessor::Insert(const PointsBatch& batch,
                                 const transform::Rigid3f& transform,
                                 Aggregation* const aggregation) {
  constexpr Color kDefaultColor = {{0, 0, 0}};
  for (size_t i = 0; i < batch.points.size(); ++i) {
    const Eigen::Vector3f camera_point = transform * batch.points[i];
    const Eigen::Array3i cell_index =
        aggregation->voxels.GetCellIndex(camera_point);
    *aggregation->voxels.mutable_value(cell_index) = true;
    bounding_box_.extend(cell_index.matrix());
    ColumnData& column_data =
        aggregation->column_data[std::make_pair(cell_index[1], cell_index[2])];
    const auto& color =
        batch.colors.empty() ? kDefaultColor : batch.colors.at(i);
    column_data.sum_r += color[0];
    column_data.sum_g += color[1];
    column_data.sum_b += color[2];
    ++column_data.count;
  }
}

void XRayPointsProcessor::Process(std::unique_ptr<PointsBatch> batch) {
  if (floors_.empty()) {
    CHECK_EQ(aggregations_.size(), 1);
    Insert(*batch, transform_, &aggregations_[0]);
  } else {
    for (size_t i = 0; i < floors_.size(); ++i) {
      if (!ContainedIn(batch->time, floors_[i].timespans)) {
        continue;
      }
      Insert(*batch, transform_, &aggregations_[i]);
    }
  }
  next_->Process(std::move(batch));
}

PointsProcessor::FlushResult XRayPointsProcessor::Flush() {
  if (floors_.empty()) {
    CHECK_EQ(aggregations_.size(), 1);
    WriteVoxels(aggregations_[0],
                file_writer_factory_(output_filename_ + ".png").get());
  } else {
    for (size_t i = 0; i < floors_.size(); ++i) {
      WriteVoxels(
          aggregations_[i],
          file_writer_factory_(output_filename_ + std::to_string(i) + ".png")
              .get());
    }
  }

  switch (next_->Flush()) {
    case FlushResult::kRestartStream:
      LOG(FATAL) << "X-Ray generation must be configured to occur after any "
                    "stages that require multiple passes.";

    case FlushResult::kFinished:
      return FlushResult::kFinished;
  }
  LOG(FATAL);
}

}  // namespace io
}  // namespace cartographer




```

本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode
