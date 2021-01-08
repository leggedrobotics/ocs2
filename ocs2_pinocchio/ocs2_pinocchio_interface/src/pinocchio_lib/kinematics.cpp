/*
 * Author: Michael Spieler
 * Date:   2020-09-08
 */

#include <ocs2_pinocchio_interface/pinocchio_lib.h>

#include <pinocchio/algorithm/kinematics.hpp>

namespace pinocchio {

template void forwardKinematics<scalar_t, 0, JointCollectionDefaultTpl>(const Model& model, Data& data,
                                                                        const Eigen::MatrixBase<Data::ConfigVectorType>& q);

}  // namespace pinocchio
