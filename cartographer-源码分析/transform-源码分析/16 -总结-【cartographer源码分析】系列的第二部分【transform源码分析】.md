

 
### 【cartographer源码分析】系列的第二部分【transform源码分析】 
现总结transform文件夹涉及到的主要功能：

## 【1】transform/rigid_transform.h：
rigid_transform.h主要定义了Rigid2 和Rigid3，并封装了2D变换和3D变换的相关函数。

---
 Rigid2 封装了2D平面网格的旋转和平移操作，方便使用2D变换。
 
- 含有2个数据成员
  * translation_代表平移向量[dx,dy]
  * Rotation2D rotation_;即Eigen::Rotation2D,代表旋转的方向角 [θ]旋转变换
.

- 提供2个构造函数
 *   Rigid2();//平移向量为单位向量[1,0]^t,旋转角度为0,debugstring()输出:[1,0,0]
 *   Rigid2(const Vector& translation, const Rotation2D& rotation);//双参构造函数，给定平移向量[dx,dy]和旋转角度0
.

- 提供4个静态成员函数
 * static Rigid2 Rotation(const double rotation) { //给定旋转角度θ,返回Rigid2,debugstring()是[0,0,θ ]
 * static Rigid2 Rotation(const Rotation2D& rotation) {//角度为θ,返回Rigid2,debugstring()是[0,0,θ ]
 * static Rigid2 Translation(const Vector& vector);//旋转角度是单位矩阵,即θ为0,debugstring()是[dx,dy,0]
 * static Rigid2<FloatType> Identity()；//静态成员函数，返回Rigid2,[0,0,0]
.
   
- 提供6个成员函数
 *  Rigid2<OtherType> cast();//按照指定的参数类型将数据成员进行类型转换
 *  const Vector& translation();// //返回平移向量[dx,dy]
 *  Rotation2D rotation();// 返回Eigen旋转矩阵 Rotation2D
 *  double normalized_angle() ;//归一化角度 ,弧度[-pi;pi]
 *  inverse() ;逆变换,[-dx’,-dy‘,-θ]
 *  string DebugString() const；//返回string形式的变换内容
.
-  提供2个友元函数
 * Rigid2<FloatType> operator*(const Rigid2<FloatType>& lhs,const Rigid2<FloatType>& rhs) ;//2个Rigid2相乘,得到第三个Rigid2,等效于连续变换2次。
 *  Rigid2<FloatType>::Vector operator*(const Rigid2<FloatType>& rigid,const typename Rigid2<FloatType>::Vector& point);//公式1的实现。

---
Rigid3是三维网格变换。使用Eigen的四元数对网格进行3D变换

-  含有2个数据成员
  * Vector translation_;//x,y,z方向上的平移向量[dx,dy,dz]
  * Quaternion rotation_;//四元数。旋转表达。
.
-  含有3个构造函数
  * Rigid3();//构造函数，DebugString()默认是[1,0,0]和[1,0,0,0]。
  * Rigid3(const Vector& translation, const Quaternion& rotation)；//构造函数，提供平移向量[dx,dy,dz]和旋转四元数参数
  * Rigid3(const Vector& translation, const AngleAxis& rotation)；//构造函数，提供平移向量[dx,dy,dz]和绕坐标轴的旋转量
.
-  含有4个静态成员函数
 * static Rigid3 Rotation(const AngleAxis& angle_axis);//静态成员函数.[dx,dy,dz]为0,只绕坐标轴旋转。
 * static Rigid3 Rotation(const Quaternion& rotation); //静态成员函数, 只旋转，不平移。
 * static Rigid3 Translation(const Vector& vector) ;//不旋转,只平移[dx,dy,dz]
 * static Rigid3<FloatType> Identity();//单位旋转，DebugString()是[0,0,0]和[1,0,0,0]。
.
-  含有4个成员函数
 * Rigid3<OtherType> cast() //类型转换
 * const Vector& translation() //获取数据成员 translation_
 * const Quaternion& rotation() //获取四元数参数
 * Rigid3 inverse()  //求逆,即逆方向旋转和平移。
 *  string DebugString() const；//返回string形式的变换内容
 .
-  提供2个友元函数
 * Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs,const Rigid3<FloatType>& rhs) //乘法操作Rigid3*Rigid3,得到Rigid3
 * Rigid3<FloatType>::Vector operator*(const Rigid3<FloatType>& rigid,const typename Rigid3<FloatType>::Vector& point)  //rigid3*Vector,得到Vector
.
- 全局函数：Eigen::Quaterniond RollPitchYaw(double roll, double pitch, double yaw);//根据x，y，z旋转角返回由roll,pathch和yaw构成的4元数

---


## 【2】transform/transform.h：
transform.h封装了多个关于3D变换的函数，包括

- 获取旋转角度值
 * FloatType GetAngle(const Rigid3<FloatType>& transform)；返回3维网格变换的四元数的角度θ，四元数q=[cos(θ/2),sin(θ/2)x,sin(θ/2)y,sin(θ/2)z]
 * T GetYaw(const Eigen::Quaternion<T>& rotation) ；返四元数yaw方向的弧度值,也就是z轴方向的弧度。
 * T GetYaw(const Rigid3<T>& transform) ；返回3D变换yaw方向(z轴)的弧度值
 . 
- 根据四元数获取旋转矩阵
 * Eigen::Matrix<T, 3, 1> RotationQuaternionToAngleAxisVector( const Eigen::Quaternion<T>& quaternion) 
.
- 绕angle-axis旋转，返回四元数。
 * Eigen::Quaternion<T> AngleAxisVectorToRotationQuaternion(const Eigen::Matrix<T, 3, 1>& angle_axis)
.
- 将3维变换投影到2维平面xy。
 * Rigid2<T> Project2D(const Rigid3<T>& transform)
 .
- 将2维变换转换为3维变换。
 * Rigid3<T> Embed3D(const Rigid2<T>& transform)


 ---
 
 
## 【3】transform/transform_interpolation_buffer.h：
TransformInterpolationBuffer类定义了离散时间段内的transform变换信息。作用与ROS的tf2函数族类似

- 1个数据成员：

 * std::deque<TimestampedTransform> deque_; //队列，元素是带时间戳的变换,存储了一段时间内的变换矩阵信息

- 6个成员函数:
 * void Push(common::Time time, const transform::Rigid3d& transform);添加变换到队列尾部,当缓冲区已满时,删除队首元素
 *  bool Has(common::Time time) const;//返回能否在给定时间内计算的插值变换。time应在early-old之间，可以插值。
 *  transform::Rigid3d Lookup(common::Time time) const;//返回time处的变换,可插值
 *  common::Time earliest_time() const;返回队列缓冲区内变换的最早时间，也就是队首元素。
 *  common::Time latest_time() const; 最晚时间，也就是队尾元素
 *  bool empty() const; 队列是否为空
 
.
  ---

关于transform文件的源码分析已经完毕，更详细细节可https://github.com/learnmoreonce/cartographer 查看注释版源码。





本文发于：
*  http://www.jianshu.com/u/9e38d2febec1
*  https://zhuanlan.zhihu.com/learnmoreonce
*  http://blog.csdn.net/learnmoreonce
*  slam源码分析微信公众号:slamcode
