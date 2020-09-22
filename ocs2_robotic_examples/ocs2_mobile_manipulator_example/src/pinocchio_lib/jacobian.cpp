/*
 * Author: Michael Spieler
 * Date:   2020-09-08
 */

#include <ocs2_mobile_manipulator_example/pinocchio_lib.h>

#include <pinocchio/algorithm/jacobian.hpp>

namespace pinocchio {

template const Data::Matrix6x& computeJointJacobians<double, 0, JointCollectionDefaultTpl>(
    const Model& model, Data& data, const Eigen::MatrixBase<Data::ConfigVectorType>& q);

template const Data::Matrix6x& computeJointJacobians<double, 0, JointCollectionDefaultTpl>(const Model& model, Data& data);

}  // namespace pinocchio
