//
// Created by rgrandia on 27.07.19.
//

#ifndef OCS2_CTRL_TRAJECTORY_H
#define OCS2_CTRL_TRAJECTORY_H

#include <Eigen/StdVector>
#include "ocs2_core/misc/LinearInterpolation.h"

namespace ocs2 {

template<typename T>
struct Trajectory {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<double> time;
  std::vector<T, Eigen::aligned_allocator<T>> data;
};

} // namespace ocs2


#endif //OCS2_CTRL_TRAJECTORY_H
