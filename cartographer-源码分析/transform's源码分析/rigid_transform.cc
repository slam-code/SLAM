

#include "cartographer/transform/rigid_transform.h"

#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "glog/logging.h"

namespace cartographer {
namespace transform {

namespace {
//typedef Matrix< double , 3 , 1> Eigen::Vector3d

//根据lua字典获取vector<double>,再转换成Vector3d
Eigen::Vector3d TranslationFromDictionary(  
    common::LuaParameterDictionary* dictionary) {
  const std::vector<double> translation = dictionary->GetArrayValuesAsDoubles();
  CHECK_EQ(3, translation.size()) << "Need (x, y, z) for translation.";
  return Eigen::Vector3d(translation[0], translation[1], translation[2]);
}

}  // namespace

/*
利用AngleAxisd可以根据3个坐标轴旋转.构成四元数
https://stackoverflow.com/questions/21412169/creating-a-rotation-matrix-with-pitch-yaw-roll-using-eigen
http://eigen.tuxfamily.org/dox/classEigen_1_1AngleAxis.html

返回根据roll,pathch和yaw构成的4元数
*/
Eigen::Quaterniond RollPitchYaw(const double roll, const double pitch,
                                const double yaw) {
  const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());//构造一个AngleAxisd对象
  const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());//同上
  const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());//同上
  return yaw_angle * pitch_angle * roll_angle; //返回4元数
}



/*
从lua配置项取得

translation:平移矩阵
rotation:旋转矩阵,w,x,y,z
*/
transform::Rigid3d FromDictionary(common::LuaParameterDictionary* dictionary) {
  const Eigen::Vector3d translation =
      TranslationFromDictionary(dictionary->GetDictionary("translation").get());

  auto rotation_dictionary = dictionary->GetDictionary("rotation");
  if (rotation_dictionary->HasKey("w")) {
    const Eigen::Quaterniond rotation(rotation_dictionary->GetDouble("w"),
                                      rotation_dictionary->GetDouble("x"),
                                      rotation_dictionary->GetDouble("y"),
                                      rotation_dictionary->GetDouble("z"));
    CHECK_NEAR(rotation.norm(), 1., 1e-9);
    return transform::Rigid3d(translation, rotation);
  } else {
    const std::vector<double> rotation =
        rotation_dictionary->GetArrayValuesAsDoubles();
    CHECK_EQ(3, rotation.size()) << "Need (roll, pitch, yaw) for rotation.";
    return transform::Rigid3d(
        translation, RollPitchYaw(rotation[0], rotation[1], rotation[2]));
  }
}

}  // namespace transform
}  // namespace cartographer
