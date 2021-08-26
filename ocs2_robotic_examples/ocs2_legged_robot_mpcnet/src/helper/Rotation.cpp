#include "ocs2_legged_robot_mpcnet/helper/Rotation.h"

namespace ocs2 {
namespace legged_robot {

matrix3_t getRotationMatrixFromEulerAngles(const vector3_t& eulerAnglesZYX) {
  matrix3_t R;
  R = Eigen::AngleAxisd(eulerAnglesZYX(0), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(eulerAnglesZYX(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(eulerAnglesZYX(2), Eigen::Vector3d::UnitX());
  return R;
}

}  // namespace legged_robot
}  // namespace ocs2
