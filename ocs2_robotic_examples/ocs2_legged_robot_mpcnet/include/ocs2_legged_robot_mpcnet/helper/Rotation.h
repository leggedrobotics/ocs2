#pragma once

#include <ocs2_legged_robot/common/Types.h>

namespace ocs2 {
namespace legged_robot {

matrix3_t getRotationMatrixFromEulerAngles(const vector3_t& eulerAnglesZYX);

}  // namespace legged_robot
}  // namespace ocs2
