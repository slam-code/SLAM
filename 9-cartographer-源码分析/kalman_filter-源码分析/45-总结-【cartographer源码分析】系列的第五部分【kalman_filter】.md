
源码可在https://github.com/learnmoreonce/SLAM 下载



 
### 【cartographer源码分析】系列的第五部分【kalman_filter】。
现总结kalman_filter文件夹涉及到的主要功能：

##  【1】kalman_filter/gaussian_distribution.h

---


GaussianDistribution定义了多个变量的高斯分布。构造函数是N*1的均值矩阵和N*N的协方差矩阵

- 数据成员:
    *  Eigen::Matrix<T, N, 1> mean_;       //N*1，均值 
    *  Eigen::Matrix<T, N, N> covariance_; //N*N。协方差

- 构造函数
* GaussianDistribution(const Eigen::Matrix<T, N, 1>& mean,
                       const Eigen::Matrix<T, N, N>& covariance)
    初始化均值和协方差


* 全局函数
  - 重载"+"加号操作符
        * GaussianDistribution<T, N> operator+(const GaussianDistribution<T, N>& lhs,
                                     const GaussianDistribution<T, N>& rhs) 
高斯+高斯=对应均值+均值,对应协方差+协方差
返回值：新高斯对象
.

 - 重载乘法运算符
       * GaussianDistribution<T, N> operator*(const Eigen::Matrix<T, N, M>& lhs,
                                     const GaussianDistribution<T, M>& rhs)
1,矩阵N*M
2,高斯分布M*M
返回值:高斯分布：N*N

---

## [2] unscented_kalman_filter.h

UnscentedKalmanFilter类是根据《Probabilistic Robotics》实现的无损卡尔曼滤波的算法，并且根据论文
 A Quaternion-based Unscented Kalman Filter for Orientation，Kraft ,E.中的算法，扩展到了处理非线性噪声和传感器。卡尔曼滤波器的操作包括两个阶段：预测与更新。在预测阶段，滤波器使用上一状态的估计，做出对当前状态的估计。在更新阶段，滤波器利用对当前状态的观测值优化在预测阶段获得的预测值，以获得一个更精确的新估计值。


- 全局函数:
 *  constexpr FloatType sqr(FloatType a)  求平方
 *  Eigen::Matrix<FloatType, N, N> OuterProduct( const Eigen::Matrix<FloatType, N, 1>& v) ；求 N*1× 1*N -> 外积,N*N
 *  void CheckSymmetric(const Eigen::Matrix<FloatType, N, N>& A) ；检查A是否是对称矩阵,A减去A的转置~=0
 *  Eigen::Matrix<FloatType, N, N> MatrixSqrt(const Eigen::Matrix<FloatType, N, N>& A)  返回对称半正定矩阵的平方根B,M=B*B

.
UnscentedKalmanFilter类

 - 使用2个别名
  *  using StateType = Eigen::Matrix<FloatType, N, 1>;           //状态矩阵N*1
  *  using StateCovarianceType = Eigen::Matrix<FloatType, N, N>; //协方差矩阵N*N

 -  数据成员
  * GaussianDistribution<FloatType, N> belief_; N*1矩阵，对N个变量的估计
  *  const std::function<StateType(const StateType& state, const StateType& delta)> add_delta_;加法操作
  *  const std::function<StateType(const StateType& origin,const StateType& target)> compute_delta_;计算偏差操作


 -  构造函数
  *  
  explicit UnscentedKalmanFilter(
      const GaussianDistribution<FloatType, N>& initial_belief,                 //参数1

      std::function<StateType(const StateType& state, const StateType& delta)>  //参数2
          add_delta = [](const StateType& state,
                         const StateType& delta) { return state + delta; },
                         
      std::function<StateType(const StateType& origin, const StateType& target)> //参数3
          compute_delta =
              [](const StateType& origin, const StateType& target) 
              
        参数1,N*1矩阵,
        参数2,stl函数对象 add_delta(默认),
        参数3,stl函数对象 compute_delta(默认),
              

 - 私有的成员函数:
  * StateType ComputeWeightedError(const StateType& mean_estimate,
                                 const std::vector<StateType>& states)；//计算带权重的偏差
  * StateType ComputeMean(const std::vector<StateType>& states) ;计算均值

 - 公有成员函数:
     *   void Predict(std::function<StateType(const StateType&)> g, const GaussianDistribution<FloatType, N>& epsilon) ；预测，在预测阶段，滤波器使用上一状态的估计，做出对当前状态的估计。
     *   void Observe( std::function<Eigen::Matrix<FloatType, K, 1>(const StateType&)> h, const GaussianDistribution<FloatType, K>& delta) ；测量/观察，滤波器利用对当前状态的观测值优化在预测阶段获得的预测值，以获得一个更精确的新估计值。
 

 -  对UnscentedKalmanFilter类的理解最好通过写一些test函数用于理解。
     * 比如在test.cc中按照给定的g和h，运行500次循环后为何state[0]和state[1]均为5？
     *  运行unscented_kalman_filter_le_test.cc: 
     
             before:0 42 #滤波前
             Predict:0 0 # 执行预测
             Observe:4.995 4.995 # 执行校正

             before:4.995 4.995 #第二次滤波前
             Predict4.995 4.995 # 执行预测
             Observe: 4.9975 4.9975 # 执行校正
            可见经过2次迭代就已经较为准确了。 

---

## 【3】pose_tracker.h

pose_tracker.h定义了根据UKF对位姿的滤波后的估计PoseTracker类
 
- 全局定义
 * typedef Eigen::Matrix3d Pose2DCovariance; //3*3矩阵
 * typedef Eigen::Matrix<double, 6, 6> PoseCovariance;// 6*6 矩阵
 * struct PoseAndCovariance {
  transform::Rigid3d pose;
  PoseCovariance covariance; //6*6
};
  * PoseAndCovariance operator*(const transform::Rigid3d& transform,
                            const PoseAndCovariance& pose_and_covariance); 
  * PoseCovariance BuildPoseCovariance(double translational_variance,
                                   double rotational_variance);
 
- PoseTracker类:
 * 类内数据结构:
        -  enum {
    kMapPositionX = 0,//位置信息{X,Y,Z}
    kMapPositionY,
    kMapPositionZ,
    kMapOrientationX,//方向信息,3
    kMapOrientationY,
    kMapOrientationZ,
    kMapVelocityX,   //速度信息,6
    kMapVelocityY,
    kMapVelocityZ,
    kDimension  //9, We terminate loops with this. 只追踪9个维度
  };

    - 类内别名:
  using KalmanFilter = UnscentedKalmanFilter<double, kDimension>;//9维的卡尔曼滤波
  using State = KalmanFilter::StateType;                         //N*1矩阵
  using StateCovariance = Eigen::Matrix<double, kDimension, kDimension>;//9*9
  using Distribution = GaussianDistribution<double, kDimension>;
  
* .lua配置信息:
    -   trajectory_builder_3d.lua:
   pose_tracker = {
      orientation_model_variance = 5e-3,
      position_model_variance = 0.00654766,
      velocity_model_variance = 0.53926,
      -- This disables gravity alignment in local SLAM.
      imu_gravity_time_constant = 1e9,
      imu_gravity_variance = 0,
      num_odometry_states = 1,
    },
* 数据成员:
 -  const proto::PoseTrackerOptions options_; //用于位姿估计的传感器特性
 -  common::Time time_;                       //测量时间
 -  KalmanFilter kalman_filter_;              //卡尔曼滤波
 -  mapping::ImuTracker imu_tracker_;         //imu跟踪
 -  mapping::OdometryStateTracker odometry_state_tracker_;//里程计跟踪

* 私有的成员函数:
 -  static Distribution KalmanFilterInit();返回初始状态的状态变量的高斯分布
 -  const Distribution BuildModelNoise(double delta_t) const;建立零均值噪声模型
 -  void Predict(common::Time time);根据当前状态预测time时刻的状态
 -  transform::Rigid3d RigidFromState(const PoseTracker::State& state); 结合imu_tracker_和state,计算位姿pose的旋转变换。

.

* 构造函数
 -  PoseTracker(const proto::PoseTrackerOptions& options, common::Time time);在给定的time时刻初始化卡尔曼滤波参数
 -  
*  公有的成员函数:
    -   GetPoseEstimateMeanAndCovariance();通过指针获取pose的旋转参数和covariance方差
    -   AddImuLinearAccelerationObservation();根据imu观测值更新
    -   AddPoseObservation(); 根据map-frame的位姿估计更新
    -   AddOdometerPoseObservation();根据里程计的map-like frame位姿估计更新
    -   common::Time time() ;最新有效时间
    -   GetBelief(common::Time time);未来某一时刻的状态估计值
    -   Eigen::Quaterniond gravity_orientation()；imu的重力方向


---
 

关于kalman_filter文件夹的源码分析已经完毕，更详细细节可https://github.com/learnmoreonce/cartographer 查看注释版源码。


本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode
