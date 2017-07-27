

 
###【cartographer源码分析】系列的第三部分【sensor源码分析】。
现总结sensor文件夹涉及到的主要功能：
---
## 【1】sensor/point_cloud.h：

- 点云数据是指在一个三维坐标系统中的一组向量的集合。
- cartographer的PointCloud是由Vector3f组成的vector即std::vector<Eigen::Vector3f> 
- PointCloudWithIntensities则是由点云和光线强度组成的struct类。
- point_cloud.h主要定义了跟点云相关的处理操作。包括4个函数
 * PointCloud TransformPointCloud(const PointCloud& point_cloud, const transform::Rigid3f& transform);根据三维网格参数转换点云
 * PointCloud Crop(const PointCloud& point_cloud, float min_z, float max_z);去掉z轴区域外的点云,返回一个新的点云
 * proto::PointCloud ToProto(const PointCloud& point_cloud);序列化
 * PointCloud ToPointCloud(const proto::PointCloud& proto);反序列化
 
---
##【2】sensor/compressed_point_cloud.h
- CompressedPointCloud点云压缩类,压缩ponits以减少存储空间，压缩后有精度损失。方法：按照block分组。提供5个函数：
 * CompressedPointCloud(const PointCloud& point_cloud) 使用点云数据初始化，并将点云压缩到  std::vector<int32> point_data_中，num_points_为点云数量
 * PointCloud Decompress() const;返回解压缩的点云
 * bool empty() const;  点云是否为空
 * size_t size() const;  点云数量
 * ConstIterator begin() const;访问点云block的迭代器
 * ConstIterator end() const;点云block的尾后迭代器

##【3】sensor/range_data.h

- RangeData定义一系列激光雷达传感器测量数据的存储结构，包括
 *  Eigen::Vector3f origin; {x0,y0,z0},sensor坐标。
 *  PointCloud returns;    反射位置{x,y,z}，表征有物体反射。
 *  PointCloud misses;     无反射,自由空间

- CompressedRangeData定义了一些用于压缩点云的存储结构，包括：
 * Eigen::Vector3f origin;
 * CompressedPointCloud returns;
 * CompressedPointCloud misses;
 

- 定义了6个全局函数：
 * proto::RangeData ToProto(const RangeData& range_data); 序列化
 * RangeData FromProto(const proto::RangeData& proto);反序列化
 * RangeData TransformRangeData(const RangeData& range_data,const transform::Rigid3f& transform);对数据进行3d变换，转换为机器坐标
 * RangeData CropRangeData(const RangeData& range_data, float min_z, float max_z);根据min_z和max_z把不在z轴范围内的点云丢弃，剪裁到给定范围
 * CompressedRangeData Compress(const RangeData& range_data);压缩,有精度丢失。
 * RangeData Decompress(const CompressedRangeData& compressed_range_Data);解压缩,有精度丢失。
.
---

##【4】sensor/Data.h
Data是针对某一类的传感器的数据的封装。

- 类内数据结构：
 * Type :
     - 传感器类型可以是{ kImu, kRangefinder, kOdometer }中的一类。imu惯性测量单元，雷达，里程计
 * Imu :
        - Eigen::Vector3d linear_acceleration; //线性加速度,m/s2
        -  Eigen::Vector3d angular_velocity;    //角速度, rad/s
 * Rangefinder:
        - Eigen::Vector3f origin; //sensor的位姿
        - PointCloud ranges;//测距点云
.
- 数据成员:

 * Type type;传感器类型,包括imu，雷达，里程计
 * common::Time time;测量时间,time
 * Imu imu;Imu测量值
 * Rangefinder rangefinder;rangefinder,测距仪测量值
 * transform::Rigid3d odometer_pose;里程计测量值
- 3个构造函数:
 *  Data(const common::Time time, const Imu& imu)//传感器类型是imu
 *  Data(const common::Time time, const Rangefinder& rangefinder)//传感器类型是雷达
 *  Data(const common::Time time, const transform::Rigid3d& odometer_pose)//传感器类型是里程计



---
##【5】sensor/ordered_multi_queue.h
ordered_multi_queue.h定义了一系列处理多个传感器的数据的类，用于接收/标识来自传感器的数据。

定义了一个QueueKey用于标识传感器数据，并将其作为OrderedMultiQueue的关键字key。

- QueueKey
    * int trajectory_id;// 轨线id;
    * string sensor_id; //传感器id

- OrderedMultiQueue
 * 用于管理多个有序的传感器数据，是有序的多队列类,每个队列有一个key,并且有一个自定义排序函数.
queues_的形式为：
key1:Queue
key2：Queue
key3：Queue

        - Queue的形式为
  struct Queue {
    common::BlockingQueue<std::unique_ptr<Data>> queue;
    Callback callback;
    bool finished = false;
  };

- OrderedMultiQueue的数据成员有:
 * common_start_time_per_trajectory_:轨迹id及对应创建轨迹时间
 * last_dispatched_time_
 * std::map<QueueKey, Queue> queues_;按照key排序的map
 * QueueKey blocker_;

-  OrderedMultiQueue内部用 using Callback =std::function<void(std::unique_ptr<Data>)> 回调函数处理来自传感器的数据。

- 提供5个公有成员函数
 *   void AddQueue(const QueueKey& queue_key, Callback callback);添加一个【队列】Queue,名称是key,以后入队的data，调用回调函数callback处理
 *     void MarkQueueAsFinished(const QueueKey& queue_key);某一key标识的【队列】Queue已经完成入队,因此不能再入队列,并在map中移除key.
 *  void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);//对某一key标识的队列Queue,压入data,data按照回调函数处理
 *   void Flush();标记全部队列都已经finished.
        -  假想数据：
        First: {0,4,5,6}
        kSecond:{0,1,3,7}
        kThird: {0,2,8}
调用之前只处理到6，调用Flush()则处理剩余的7,8。如果不调用Flush()，则析构时会出错
.
 *   QueueKey GetBlocker() const;返回阻塞的队列(意为该队列对应的sensor的data未到达)
- 还有若干个私有的成员函数用于完成上面的功能。
- 正确理解OrderedMultiQueue的功能有助于了解cartographer内部是如何管理来自多个传感器的多个数据，并充分考虑时间有序。具体细节可以查看注释ordered_multi_queue_le_test.cc

----

## 【6】 sensor/collator.h
Collator,采集者，抽象了设备采集器。将多传感器采集的数据归并到轨迹上。只有一个默认构造函数，有2个数据成员

- Collator
 * 数据成员:
        -  std::unordered_map<int, std::vector<QueueKey>> queue_keys_;
                 int为传感器id,vector是id+sensor组成的QueueKey
        - OrderedMultiQueue queue_;多个key构成的多队列
 * 成员函数:
      -   void AddTrajectory(int trajectory_id,const std::unordered_set<string>& expected_sensor_ids,
                     Callback callback);
    添加一个轨迹线,接受有序的传感器数据,并使用callback回调处理data。  一个轨迹线对应多个传感器数据:id ->unordered_set

     -  void FinishTrajectory(int trajectory_id);标记轨迹线已经完成采样.
     
     -   void AddSensorData(int trajectory_id, const string& sensor_id,                  std::unique_ptr<Data> data);添加一个传感器id对应的数据data,数据必须按时间排序
     -   void Flush(); //刷新队列中的data
     -   int GetBlockingTrajectoryId() const;返回阻塞的轨迹id,此种情况多见于某一传感器持久未采集data，造成ordered_multi_queue阻塞。

---

##【7】 sensor/voxel_filter.h
voxel_filter.h定义了与3维网格grid体素的滤波相关的数据结构和相关函数。作用：将点云转换为稀疏体素grid表达。以节省空间

---

体素滤波器,对每一个体素voxel,采用第一个point代替该体素内的所有points

-  VoxelFilter类：
  * 数据成员:
        -  mapping_3d::HybridGridBase<uint8> voxels_;//以体素表示的网格，Grid/Pixel 表征一系列的Cell，每个Cell有一个size，piont以size为间距分布，实现稀疏表示。Cell的数量和即为稀疏表达的point的数量和。
        -    PointCloud point_cloud_; //网格内的点云
  * 构造函数 VoxelFilter(float size);初始化网格voxels_的大小
  *成员函数:
        -  void InsertPointCloud(const PointCloud& point_cloud);将点云插入体素网格中
        -  const PointCloud& point_cloud() const;返回表达occupied 体素的点云

---
- 全局函数CreateAdaptiveVoxelFilterOptions(),根据pb.h配置文件设置Options参数
  * message AdaptiveVoxelFilterOptions { 
  optional float max_length = 1; 
  optional float min_num_points = 2;
  optional float max_range = 3;
}
 * 2d:adaptive_voxel_filter = {
      max_length = 0.9,     //voxel_的大小edge的最大值
      min_num_points = 100, //voxel_最多“占据”的points数量
      max_range = 50.,
    },

    * 
- AdaptiveVoxelFilter类：
 *
  * 构造函数 AdaptiveVoxelFilter(const proto::AdaptiveVoxelFilterOptions& options);//根据配置文件设置自适应体素滤波的options
  *   PointCloud Filter(const PointCloud& point_cloud) const;对点云进行体素滤波,返回过滤后的点云

---
## 【8】sensor/configuration.h 

configuration.h 主要配置了和传感器设备相关的参数。
提供4个全局函数:

 - Sensor CreateSensorConfiguration(common::LuaParameterDictionary* parameter_dictionary);从sensor配置文件解析sensor的数据参数。主要是sensor到机器坐标的转换
 - Configuration CreateConfiguration(common::LuaParameterDictionary* parameter_dictionary)//求得多个sensor的配置的集合。
 - bool IsEnabled(const string& frame_id, const sensor::proto::Configuration& sensor_configuration);//系统是否支持某一传感器。
 - transform::Rigid3d GetTransformToTracking(const string& frame_id,const sensor::proto::Configuration& sensor_configuration); 将sensor采集的data经过3d坐标变换为机器坐标。


---

关于sensor文件夹的源码分析已经完毕，更详细细节可https://github.com/learnmoreonce/cartographer 查看注释版源码。




本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode




