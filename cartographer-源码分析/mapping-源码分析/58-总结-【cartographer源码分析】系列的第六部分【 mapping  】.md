

 
###【cartographer源码分析】系列的第六部分【 mapping】。
现总结 mapping 文件夹涉及到的主要功能：

## 【1】 mapping/probability_values.h

probability_values.h定义了一系列与概率相关的函数--多个用于计算概率的mapping命名空间下的全局函数。cartographer所有的概率不是以0-1.0表示，而是通过浮点数到整数的映射： [1, 32767]，避免浮点数运算。

- 全局函数

        *  inline float Odds(float probability) {          //论文公式(2),求胜负比。y=x/(1-x)
             return probability / (1.f - probability);    }
             
    
        *  inline float ProbabilityFromOdds(const float odds) { //求概率，即x=y/(1+y)
            return odds / (odds + 1.f);    }
            
    
            * constexpr float kMinProbability = 0.1f;//p最小是0.1
        
            * constexpr float kMaxProbability = 1.f - kMinProbability;//最大是0.9
            
            * inline float ClampProbability(const float probability);限制概率p在[0.1,0.9]之间
            
            * constexpr uint16 kUnknownProbabilityValue = 0;//标记未初始化的概率
            
            * constexpr uint16 kUpdateMarker = 1u << 15;// 32768
            
            * inline uint16 ProbabilityToValue(const float probability) 将概率p映射为整数Value[1,32767]
            
            * extern const std::vector< float>* const kValueToProbability;   声明，定义在.cc文件。vector是value到p的映射
            
            * inline float ValueToProbability(const uint16 value);映射 [1,32767]->[0.1,0.9]
        
       * std::vector< uint16> ComputeLookupTableToApplyOdds(float odds);//2份value:前一半对应没有hit，后一半对应hit。
            
            
            之前没有hit过，则没有update，按论文公式(2)计算：
            求p,
            求[1,32767],
            求[1,32767]+32768
            push_back()
            
            之前有hit过，则有update，按论文公式(3)计算：
            求(*kValueToProbability)[cell]->[0.1,0.9]即原始p
            求p'=odds*Odds(p)
            求p'映射到[1,32767]
            push_back():[1,32767]+32768.


---
##【2】id.h
该文件定义了一系列用于标记轨迹的数据结构，包括：

 - struct NodeId
            
                {
              int trajectory_id;
              int node_index; }
每一个轨迹trajectory上有多个节点node。节点标号:轨迹id+{0,1,2,...}.

 -  struct SubmapId {
          
          int trajectory_id;
          int submap_index;  }
  重建全局地图global map时，是由多个submap组成。submap标号: 轨迹id+ {0,1,2,3...}


 - NestedVectorsById 
    *   作用：嵌套存储多条轨迹线上的数据
    *   数据成员:vector< vector<ValueType> > data_;
    *   对外提供4个操作：
     -      IdType Append(int trajectory_id, const ValueType& value) 根据轨迹id和data添加数据
     -      const ValueType& at(const IdType& id)查询某轨迹对应的vector的data
     -      num_indices(),某轨迹对应的数据的大小
     -      int num_trajectories()  轨迹数量
     -  


## 【3】imu_tracker.h
ImuTracker类利用来自imu测量仪器的数据对机器的位姿pose的方向角进行跟踪和预测。x,y轴不会drift漂移。z轴可能会产生漂移。(因为绕z轴旋转角有累计误差)

-  作用：使用来自imu的角速度+加速度用于跟踪pose的orientation
-  数据成员：
 *   const double imu_gravity_time_constant_;    // g，10.0
 *   common::Time time_;                         //最新一次测量时间
 *   common::Time last_linear_acceleration_time_;//加速度测量时间
 *   Eigen::Quaterniond orientation_;            //pose的方向角
 *   Eigen::Vector3d gravity_vector_;            //加速度测量的方向
 *   Eigen::Vector3d imu_angular_velocity_;      //角速度

- 成员函数:
 *  void Advance(common::Time time);   系统时间增加t，更新方向角
 *  void AddImuLinearAccelerationObservation()   //更新imu测量得到的加速度。
 *  void AddImuAngularVelocityObservation()     //更新imu测量得到的角速度
 *  Eigen::Quaterniond orientation();返回目前估计pose的方向角。

## 【4】 odometry_state_tracker.h
 odometry_state_tracker.h定义了与里程计相关的跟踪接口
 
- .lua设置：
imu_gravity_time_constant = 10.,
num_odometry_states = 1000,
- OdometryState类：

  * 含3个数据成员
       -      common::Time time ，时间
       -      transform::Rigid3d odometer_pose，里程计的位置
       -    transform::Rigid3d state_pose，状态位置

- OdometryStateTracker类：
 *  作用：
 *   数据成员：
        -    OdometryStates odometry_states_;记录多个里程计状态，是一个双端队列deque
        -     size_t window_size_;滑动窗大小,即队列大小
 *  构造函数 explicit OdometryStateTracker(int window_size);构造函数初始化滑动窗大小.
 *  成员函数:
        - void AddOdometryState(const OdometryState& odometry_state);添加新的里程计状态,超出滑动窗大小时,旧的删除
        -  bool empty() const;队列是否为空
        -   const OdometryState& newest() const;里程计最新的状态
        


##【5】detect_floors.h
detect_floors.h定义了关于3D扫描楼层的数据结构。

- struct Timespan {
  common::Time start;
  common::Time end;
};
 Timespan表征扫描的时间范围。
- struct Floor {
  std::vector< Timespan> timespans;
  double z; //z轴的中值
};
一个楼层对应多个扫描timespan：有可能重复的扫描多次
但只有一个高度z。

- std::vector<Floor> DetectFloors(const proto::Trajectory& trajectory);
使用启发式搜索寻找building的不同楼层的z值。

---

## 【6】 submaps.h

- 全局函数
  * inline float Logit(float probability); //求odds(p)的log对数
  * inline uint8 ProbabilityToLogOddsInteger(const float probability) 将[0.1,0.9]映射为0-255之间的数

-  Submap
   * 数据成员
     -       const transform::Rigid3d local_pose;子图的位姿
     -       int num_range_data = 0;插入的数据量
     -       const mapping_2d::ProbabilityGrid* finished_probability_grid = nullptr;完成建图的概率网格。 当子图不再变化时，指向该子图的概率分布网格。


-  Submaps 
    *   Submaps是一连串的子图,初始化以后任何阶段均有2个子图被当前scan point影响：old submap 用于now match,new submap用于next match,一直交替下去。一旦new submap有足够多的scan point，那么old submap不再更新。此时new submap变为old，用于 scan-to-map匹配。  
    *   Submaps是虚基类,没有数据成员,只提供成员函数用于实现接口。
    *   成员函数抽象接口：
      -  int matching_index() const;最新的初始化的子图索引，用于scan-to-map-match，size() - 2;
      -   std::vector< int> insertion_indices() const;待插入的子图的索引， {size() - 2, size() - 1};
      -    virtual const Submap* Get(int index) const;纯虚函数，按索引返回子图指针。
      -    virtual int size() const ；子图数量
      -    static void AddProbabilityGridToResponse();将子图对应的概率网格序列化到proto文件中
      -  virtual void SubmapToProto(); 将对应的子图序列化到proto文件中

---

## 【7】trajectory_node.h
- 轨迹节点TrajectoryNode类，含有一个内部类ConstantData。
-   TrajectoryNode作用：在连续的轨迹上采样一些离散的点用于key frame，标识pose frame。
-   类内数据结构
  * ConstantData  common::Time time;
           
             sensor::RangeData range_data_2d;//测量得到的2D range数据
            sensor::CompressedRangeData range_data_3d;//测量得到的3D range数据
            int trajectory_id;//节点所属id
            transform::Rigid3d tracking_to_pose; //tracking frame 到 pose frame的矩阵变换。
          };

- TrajectoryNode类
 * 数据成员：
      - const ConstantData* constant_data;
      - transform::Rigid3d pose;
 * 成员函数common::Time time();返回测量时间


## 【8】trajectory_connectivity.h
TrajectoryConnectivity用于解决不同轨迹线的连通性问题.多条轨迹构成一颗森林，而相互联通的轨迹应该合并。

- 数据成员：
    *    common::Mutex lock_;
    *      std::map < int, int > forest_ ；不同轨迹线组成的森林
    *        std::map < std::pair< int, int>, int> connection_map_ ；直接链接的轨迹线
- 成员函数：
    *  void Add(int trajectory_id) EXCLUDES(lock_);添加一条轨迹线，默认不连接到任何轨迹线
    *   void Connect(int trajectory_id_a, int trajectory_id_b) ；将轨迹a和轨迹b联通
    *   TransitivelyConnected(int trajectory_id_a, int trajectory_id_b)；判断是否处于同一个连通域
    *     int ConnectionCount(int trajectory_id_a, int trajectory_id_b)；返回直接联通的数量
    *       std::vector < std::vector < int > > ConnectedComponents()；:由联通分量id组成的已联通分类组
- 全局函数
  *     proto::TrajectoryConnectivity ToProto(
    std::vector< std::vector< int>> connected_components);
    编码已联通成分到proto文件
  * ConnectedComponent FindConnectedComponent()//返回连接到联通id的所有联通分量。

---
##【9】 trajectory_builder.h

TrajectoryBuilder:虚基类,提供多个抽象接口。没有数据成员

-   作用：根据轨迹Builder收集data。
-   类内数据结构：
    * PoseEstimate{
    
            common::Time time = common::Time::min();测量时间
            transform::Rigid3d pose = transform::Rigid3d::Identity();世界坐标转换
            sensor::PointCloud point_cloud; 子图所在的点云
}
-   虚函数：
 *    virtual const Submaps* submaps() =0;一系列子图
 *    virtual const PoseEstimate& pose_estimate() const = 0;子图位姿及其采集的点云
 *    virtual void AddSensorData(const string& sensor_id,
                             std::unique_ptr< sensor::Data> data) = 0;
        根据sensor_id添加data，虚函数。
.
-   非虚函数：下面3个函数都是非虚函数。分别是添加雷达/imu/里程计的data。
 * AddRangefinderData();
 * AddImuData();
 * AddOdometerData();
        -   参数：
1),sensor_id,标识传感器。
2),time 测量时间
3),PointCloud/Vector3d  /Rigid/Rigid3d 测量得到的数据

---

##【10】collated_trajectory_builder.h
 
 ---
 
 CollatedTrajectoryBuilder类继承自TrajectoryBuilder

-   作用：使用Collator处理从传感器收集而来的数据.并传递给GlobalTrajectoryBuilderInterface。

- 数据成员
    *   sensor::Collator* const sensor_collator_; 传感器收集类实例
    *   const int trajectory_id_;                 轨迹id
    *   std::unique_ptr< GlobalTrajectoryBuilderInterface >  wrapped_trajectory_builder_;全局的建图接口
    *   std::chrono::steady_clock::time_point last_logging_time_; 上一次传递数据时间
    *   std::map< string, common::RateTimer<>> rate_timers_;频率

- 成员函数,重写了父类的接口
 *   const Submaps* submaps() const override;
 *   const PoseEstimate& pose_estimate() const override;
 *   void AddSensorData(const string& sensor_id,std::unique_ptr<sensor::Data> data) override;

---
## [11] sparse_pose_graph.h
    
SparsePoseGraph:稀疏位姿图模型,虚基类,提供多个抽象接口,不可拷贝/赋值

- 作用： 
    稀疏图用于闭环检测
-   类内数据结构：
    -   struct Pose {
 
             transform::Rigid3d zbar_ij;
              double translation_weight;
              double rotation_weight;
    };
    -    struct Constraint { //约束
    struct Pose {

                mapping::SubmapId submap_id;  // 'i' in the paper.
                mapping::NodeId node_id;      // 'j' in the paper.
                Pose pose;
                enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
  };

- 虚函数，抽象接口:
    *     virtual void RunFinalOptimization() = 0;计算优化后的位姿估计
    *     virtual std::vector< std::vector<int>> GetConnectedTrajectories() = 0;获取已连接的轨迹集合
    *     virtual std::vector< transform::Rigid3d> GetSubmapTransforms( int trajectory_id) = 0;        获取优化后的位姿估计的3D变换
    *  virtual transform::Rigid3d GetLocalToGlobalTransform(int trajectory_id) = 0;获取局部图的3D变换
    *   virtual std::vector< std::vector< TrajectoryNode>> GetTrajectoryNodes() = 0;优化后的轨迹线
    *   virtual std::vector< Constraint> constraints() = 0; 获取约束集

---

## [12] map_builder.h

---

MapBuilder类,建图,不可拷贝/赋值

-  作用：MapBuilder类和TrajectoryBuilder类即真正的开始重建局部子图submaps,并且采集稀疏位姿图用于闭环检测。
-   数据成员:
    *  const proto::MapBuilderOptions options_; // 建图选项
    *   common::ThreadPool thread_pool_;         //线程数量，不可变。
    *  std::unique_ptr< mapping_2d::SparsePoseGraph> sparse_pose_graph_2d_; //稀疏2D图
    *  std::unique_ptr< mapping_3d::SparsePoseGraph> sparse_pose_graph_3d_; //稀疏3D图
    *  mapping::SparsePoseGraph* sparse_pose_graph_;  //稀疏位姿

  sensor::Collator sensor_collator_;  //收集传感器采集的数据
  std::vector< std::unique_ptr< mapping::TrajectoryBuilder> > trajectory_builders_;//轨迹线集合

-  成员函数:
 *  int AddTrajectoryBuilder()根据传感器id和options新建一个轨迹线，返回轨迹线的索引
 *  TrajectoryBuilder* GetTrajectoryBuilder(int trajectory_id);根据轨迹id返回指向该轨迹的TrajectoryBuilder对象指针。
 *  void FinishTrajectory(int trajectory_id);标记该轨迹已完成data采集，后续不再接收data
 *  int GetBlockingTrajectoryId() const;阻塞的轨迹，常见于该条轨迹上的传感器迟迟不提交data。
 *  proto::TrajectoryConnectivity GetTrajectoryConnectivity();获得一系列轨迹的连通域
 *   string SubmapToProto();把轨迹id和子图索引对应的submap，序列化到文件
 *     int num_trajectory_builders() const;在建图的轨迹数量


-   

---
关于 mapping文件夹的源码分析已经完毕，更详细细节可https://github.com/learnmoreonce/cartographer 查看注释版源码。
markdown文件可在https://github.com/learnmoreonce/SLAM 下载





本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode




