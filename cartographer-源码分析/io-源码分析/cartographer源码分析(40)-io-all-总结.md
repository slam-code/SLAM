
###【cartographer源码分析】系列的第四部分【io源码分析】。

现总结io文件夹涉及到的主要功能：

---
##【1】file_writer.h
file_writer.h定义了多个用于文件写入的类

- FileWriter 
 * FileWriter是负责文件写入的虚基类没有数据成员.只提供一系列抽象接口.
 * 包括3个抽象接口：
        - WriteHeader(),写入文件head
        - Write(),写入数据
        - Close(),关闭文件
- StreamFileWriter
 * StreamFileWriter 文件流写入类,继承自FileWriter
 * 数据成员只有一个std::ofstream out_负责将文件写入磁盘.
 * 包括3个成员函数：
        - WriteHeader(),写入文件head
        - Write(),写入数据
        - Close(),关闭文件

- 全局函数:
 * using FileWriterFactory =
    std::function< std::unique_ptr< FileWriter >(const string& filename)>;
工厂模式,
创建一个FileWriter对象,由智能指针管理生命期,
返回值是std::unique_ptr<FileWriter>,
函数参数是string.


---
## 【2】io/points_batch.h：
* PointsBatch类是对多个点云point的抽象.这些point在由同一时刻,同一机器人坐标地点的传感器采集而得。
数据成员主要描述了point的特性.
PointsBatch数据成员:
 - common::Time time; point采集时间.
 - Eigen::Vector3f origin; sensor的世界坐标, 传感器位姿
 - string frame_id;关键帧的id
 - int trajectory_index;轨迹线id
 - std::vector<Eigen::Vector3f> points;point的几何参数,vector<{x,y,z}>
 -  std::vector<float> intensities;光强
 - std::vector<Color> colors;point的rgb值

* 全局函数
  - void RemovePoints(std::vector<int> to_remove, PointsBatch* batch);按照to_temove中的索引,在batch中移除某些point.

## 【3】流水线-points_processor

整个IO文件夹实现了对点云数据的读取和存储，并且为了模块化，cartographer使用流水线作业的方式对点云进行处理(pipeline)。不同的.h文件抽象了流水线不同的处理器processor。并且类似于链表，每个在流水线上的processor都含有一个Next指针，执行下一阶段的processor。以此来执行作业。在assets_writer_backpack_2d.lua文件中有各个pipeline的处理流程.

## 【4】io/points_processor.h
---
points_processor.h文件夹定义了一个抽象基类PointsProcessor，抽象了所有在流水线上的processor的公有接口。提供一种批量处理points的方法。

- 类内数据结构:

 *   enum class FlushResult {
    kRestartStream,//重启流水线
    kFinished,//已完成作业
  };
 用于表达本处理器的当前状态，枚举值

- 2个抽象接口:
 *  virtual void Process(std::unique_ptr<PointsBatch> points_batch) =0;纯虚函数，Process()负责对PointsBatch进行处理
 * Flush()刷新点云数据.
 ---

## 【5】min_max_range_filtering_points_processor

---

MinMaxRangeFiteringPointsProcessor是PointsProcessor的第一个子类(1).

- .lua配置,可修改
 *   pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 1.,
      max_range = 60.,
    },

- 功能:继承自PointsProcessor点云虚基类.距离过滤器,L2距离不在Min-Max范围内的都过滤掉.
- 数据成员:
 * min_range_ 最小范围。
 * max_range_ 最大范围。
 * PointsProcessor* const next_:完成本轮Processor,接下来进行下一次Processor.
 * kConfigurationFileActionName="min_max_range_filter" ：每个处理器processor用于标识自己的名称
.
- 构造函数
 *  MinMaxRangeFiteringPointsProcessor(double min_range, double max_range,
                                     PointsProcessor* next); 指定过滤范围和流水线next指针。
 - 成员函数:
  * Process(std::unique_ptr<PointsBatch> batch) ;//点云处理,删除[min,max]以外的point，并把数据传递到下一轮Processor处理。
  * FlushResult Flush() override;调用next_->Flush():父类指针调用Flush(),但在运行时才会决定是否调用子类的Flush()，即达到动态绑定的效果。
 ---

## 【6】counting_points_processor.h
 ---

CountingPointsProcessor是PointsProcessor的第二个子类(2).

- .lua配置,可修改
 *   pipeline = {
      action = "dump_num_points",
    }
- 功能:继承自PointsProcessor点云虚基类.记录有多少 point被输出处理
- 数据成员:
 * num_points_记录points数量
 * PointsProcessor*  next_:完成本轮Processor,接下来进行下一次Processor.
 * kConfigurationFileActionName ="dump_num_points"：每个处理器processor用于标识自己的名称
.
- 构造函数
 *  CountingPointsProcessor(PointsProcessor* next); 指定流水线next指针。
 - 成员函数:
  * Process(std::unique_ptr<PointsBatch> batch)；//不处理points,而是将num_points_加上batch.size()，即统计点云数据。然后直接流水线到下一PointsProcessor
  * FlushResult Flush() override;调用next_->Flush()，依据下一阶段的状态重置本阶段的状态。
 ---

## 【7】xray_points_processor.h
 ---

XRayPointsProcessor是PointsProcessor的第三个子类(3).

- .lua配置,可修改
 *  VOXEL_SIZE = 5e-2

 * YZ_TRANSFORM =  {
  translation = { 0., 0., 0. },
  rotation = { 0. , 0., math.pi, },
}
 *{
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_yz_all",
      transform = YZ_TRANSFORM,
    }


- 类内数据结构:
 *   struct ColumnData {
    double sum_r = 0.;
    double sum_g = 0.;
    double sum_b = 0.;
    uint32_t count = 0;
  };
  *  struct Aggregation {
    mapping_3d::HybridGridBase<bool> voxels;
    std::map<std::pair<int, int>, ColumnData> column_data;
  };

- 功能:继承自PointsProcessor点云虚基类.将x射线以体素为voxel_size写入image。
- 数据成员:
 * PointsProcessor*  next_:完成本轮Processor,接下来进行下一次Processor.
 * kConfigurationFileActionName ="write_xray_image"：每个处理器processor用于标识自己的名称
 * FileWriterFactory file_writer_factory_; //负责文件写入的工厂
 * std::vector<mapping::Floor> floors_; //楼层
 * const string output_filename_;       //文件名称
 * const transform::Rigid3f transform_; //点云涉及的变换
 * std::vector<Aggregation> aggregations_; //聚集数据
 * Eigen::AlignedBox3i bounding_box_;//边界框


- 构造函数
 *   XRayPointsProcessor(double voxel_size, const transform::Rigid3f& transform,
                      const std::vector<mapping::Floor>& floors,
                      const string& output_filename,
                      FileWriterFactory file_writer_factory,
                      PointsProcessor* next);
  初始化数据成员，并指定流水线next指针。
 - 成员函数:
  * Process(std::unique_ptr<PointsBatch> batch)；//主逻辑，将点云写入.png文件，然后流水到下一处理器
PointsProcessor
  * FlushResult Flush() override;调用next_->Flush()，依据下一阶段的状态重置本阶段的状态。
 ---


## 【8】intensity_to_color_points_processor.h
 ---

ColoringPointsProcessor是PointsProcessor的第四个子类(4).处理强度到color point 的强度变换类

- .lua配置,可修改
 *   pipeline = {
      action = "intensity_to_color",
      min_intensity = 0.,
      max_intensity = 4095.,
    },

- 功能:继承自PointsProcessor点云虚基类. 功能:将光强度转换为color

- 数据成员:
 * const float min_intensity_;
 * const float max_intensity_;
 * const string frame_id_;只有相同的id才将光强度转换为color。horizontal_laser_link或者vertical_laser_link
 * PointsProcessor*  next_:完成本轮Processor,接下来进行下一次Processor.
 * kConfigurationFileActionName ="intensity_to_color"：每个处理器processor用于标识自己的名称
 .

- 构造函数
  *   IntensityToColorPointsProcessor(float min_intensity, float max_intensity,
                                  const string& frame_id,
                                  PointsProcessor* next);;初始化min和max，frame_id和流水线next指针。

- 成员函数:
    * Process(std::unique_ptr<PointsBatch> batch)；//成员函数执行转换:('intensity' - min ) / (max - min) * 255 然后直接流水线到下一PointsProcessor
    * FlushResult Flush() override;调用next_->Flush()，依据下一阶段的状态重置本阶段的状态。

     ---

## 【9】ply_writing_points_processor.h
 ---
PlyWritingPointsProcessor是PointsProcessor的第五个子类(5).
PlyWritingPointsProcessor负责将点云point以PLY的格式写入磁盘.

- .lua配置,可修改
 *   pipeline =   {
      action = "write_ply",
      filename = "points.ply",
    }
- 功能:继承自PointsProcessor点云虚基类.负责将点云point以PLY的格式写入磁盘.

- 数据成员:
 * num_points_记录points数量
 *  bool has_colors_;//是否是RGB
 * PointsProcessor*  next_:完成本轮Processor,接下来进行下一次Processor.
 * kConfigurationFileActionName ="dump_num_points"：每个处理器processor用于标识自己的名称
 .

- 构造函数
  * PlyWritingPointsProcessor(std::unique_ptr<FileWriter> file,
                            PointsProcessor* next); 指定文件写入的工厂和流水线next指针。

- 成员函数:
    * Process(std::unique_ptr<PointsBatch> batch)；//按照ply格式写点云信息 。然后直接流水线到下一PointsProcessor
    * FlushResult Flush() override;调用next_->Flush()，依据下一阶段的状态重置本阶段的状态。


     ---

## 【10】coloring_points_processor.h
 ---

ColoringPointsProcessor是PointsProcessor的第六子类(6).

- .lua配置,可修改
 *   pipeline =   {
      action = "color_points",
      frame_id = "horizontal_laser_link",
      color = { 255., 0., 0. },
    }

- 功能:继承自PointsProcessor点云虚基类. 用固定的Color填充PointsBatch的Color分量。着色

- 数据成员:
 * color_：rgb值，color一般为[255,0,0],[0,255,0]
 * frame_id_:只有相同的id才填充Color。horizontal_laser_link或者vertical_laser_link
 * PointsProcessor*  next_:完成本轮Processor,接下来进行下一次Processor.
 * kConfigurationFileActionName ="color_points"：每个处理器processor用于标识自己的名称
 .

- 构造函数
  * ColoringPointsProcessor(const Color& color, const string& frame_id,
                          PointsProcessor* next);初始化 color_，frame_id和流水线next指针。

- 成员函数:
    * Process(std::unique_ptr<PointsBatch> batch)；//着色，只对相同的frame_id_处理。然后直接流水线到下一PointsProcessor
    * FlushResult Flush() override;调用next_->Flush()，依据下一阶段的状态重置本阶段的状态。


---
## 【11】fixed_ratio_sampling_points_processor.h
---

FixedRatioSamplingPointsProcessor是PointsProcessor的第七个子类(7).
FixedRatioSamplingPointsProcessor是采样过滤器

- .lua配置,可修改
 *  sampling_ratio = 0.3,
- 功能:继承自PointsProcessor点云虚基类.该class只让具有固定采样频率的点通过,其余全部 remove.sampling_ratio=1,即无任何操作,全通滤波.

- 数据成员:
 * const double sampling_ratio_;采样率
 * std::unique_ptr<common::FixedRatioSampler> sampler_;具有固定采样率的采样函数
 * PointsProcessor*  next_:完成本轮Processor,接下来进行下一次Processor.
 * kConfigurationFileActionName ="fixed_ratio_sampler"：每个处理器processor用于标识自己的名称
 .

- 构造函数
  * 
  FixedRatioSamplingPointsProcessor(double sampling_ratio,
                                    PointsProcessor* next);初始化采样率和流水线next指针。

- 成员函数:
    * Process(std::unique_ptr<PointsBatch> batch)；//根据采样率采集点云，不在采样点上的全部删除。
    * FlushResult Flush() override;调用next_->Flush()，依据下一阶段的状态重置本阶段的状态。

---

## 【12】 outlier_removing_points_processor.h
---
OutlierRemovingPointsProcessor是PointsProcessor的第八个子类(8).

- .lua配置,可修改
 *  VOXEL_SIZE = 5e-2
- 功能:继承自PointsProcessor点云虚基类.体素过滤器,Remove有移动痕迹的体素。只保留没有移动的体素
- 类内数据结构:
  *   struct VoxelData {
    int hits = 0;
    int rays = 0;
  };
  * enum class State {
    kPhase1,
    kPhase2,
    kPhase3,
  };
- 数据成员:
 * const double voxel_size_;//体素大小
 * State state_;
 * mapping_3d::HybridGridBase<VoxelData> voxels_;包含多个体素的网格Grid。
 * PointsProcessor*  next_:完成本轮Processor,接下来进行下一次Processor.
 * kConfigurationFileActionName="voxel_filter_and_remove_moving_objects"：每个处理器processor用于标识自己的名称
 .

- 构造函数
  * OutlierRemovingPointsProcessor(double voxel_size, PointsProcessor* next);初始化体素大小和流水线next指针。

- 成员函数:
    * Process(std::unique_ptr<PointsBatch> batch)；//删除移动的体素。
    * FlushResult Flush() override;调用next_->Flush()，依据下一阶段的状态重置本阶段的状态。

---

## 【13】pcd_writing_points_processor.h
---
PcdWritingPointsProcessor是PointsProcessor的第九个子类(9).

- .lua配置,可修改
 *  VOXEL_SIZE = 5e-2
- 功能:继承自PointsProcessor点云虚基类.将点云按照pcd格式存储在pcd文件中.
- 类内数据结构:
  *   struct VoxelData {
    int hits = 0;
    int rays = 0;
  };
  * enum class State {
    kPhase1,
    kPhase2,
    kPhase3,
  };
- 数据成员:
 * int64 num_points_; //点云数量
 * bool has_colors_; //是否是彩色
 * PointsProcessor*  next_:完成本轮Processor,接下来进行下一次Processor.
 * kConfigurationFileActionName="write_pcd"：每个处理器processor用于标识自己的名称
 .

- 构造函数
  *   PcdWritingPointsProcessor(std::unique_ptr<FileWriter> file_writer,
                            PointsProcessor* next);初始化一个文件类名和流水线next指针。

- 成员函数:
    * Process(std::unique_ptr<PointsBatch> batch)；//负责将点云按照PCD格式写入文件
    * FlushResult Flush() override;调用next_->Flush()，依据下一阶段的状态重置本阶段的状态。

---
##【14】null_points_processor.h

NullPointsProcessor是PointsProcessor的第十个子类(10)
空操作类。通常用于pipline的尾端，丢弃所有的点云，表示整个处理流程已经完毕。

- 成员函数:
    * Process(std::unique_ptr<PointsBatch> batch)；不作任何事情
    * FlushResult Flush() override;返回"kFinished"状态，此操作会传递给上一个流水线。


---

关于io文件夹的源码分析已经完毕，更详细细节可https://github.com/learnmoreonce/cartographer 查看注释版源码。
markdown文件可在https://github.com/learnmoreonce/SLAM 下载





本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode




