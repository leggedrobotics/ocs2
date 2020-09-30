/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <ocs2_pinocchio/cost/SelfCollisionCost.h>
// Todo(perry) why does this header need to come after the SelfCollisionCost.h?
#include <ocs2_pinocchio/CppAdHelpers.h>

#include <pinocchio/multibody/geometry.hpp>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionCost::SelfCollisionCost(ocs2::PinocchioInterface<scalar_t> pinocchioInterface,
                                     ocs2::PinocchioGeometryInterface geometryInterfaceSelfCollision, scalar_t minimumDistance, scalar_t mu,
                                     scalar_t delta)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      pinocchioGeometrySelfCollisions_(std::move(geometryInterfaceSelfCollision)),
      minimumDistance_(minimumDistance),
      relaxedBarrierPenalty_(mu, delta) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionCost::SelfCollisionCost(const SelfCollisionCost& rhs)
    : pinocchioInterface_(rhs.pinocchioInterface_),
      pinocchioGeometrySelfCollisions_(rhs.pinocchioGeometrySelfCollisions_),
      minimumDistance_(rhs.minimumDistance_),
      relaxedBarrierPenalty_(rhs.relaxedBarrierPenalty_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SelfCollisionCost::cost(scalar_t t, const vector_t& x, const vector_t& u) {
  //  std::cout << "Cost called" << std::endl;
  const std::vector<hpp::fcl::DistanceResult> results = pinocchioGeometrySelfCollisions_.computeDistances(x);

  vector_t violations = vector_t::Zero(results.size());
  for (size_t i = 0; i < results.size(); ++i) {
    const hpp::fcl::DistanceResult& result = results[i];
    violations[i] = result.min_distance - minimumDistance_;
  }

  return relaxedBarrierPenalty_.penaltyCost(violations);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SelfCollisionCost::finalCost(scalar_t t, const vector_t& x) {
  return scalar_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation SelfCollisionCost::costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  const std::vector<hpp::fcl::DistanceResult> results = pinocchioGeometrySelfCollisions_.computeDistances(x);

  VectorFunctionQuadraticApproximation distanceQuadraticApproximation;
  distanceQuadraticApproximation.f = vector_t(results.size());
  distanceQuadraticApproximation.dfdx.resize(results.size(), x.size());
  distanceQuadraticApproximation.dfdu = matrix_t::Zero(results.size(), u.size());
  distanceQuadraticApproximation.dfdxx = matrix_array_t(results.size(), matrix_t::Zero(x.size(), x.size()));
  distanceQuadraticApproximation.dfdux = matrix_array_t(results.size(), matrix_t::Zero(x.size(), u.size()));
  distanceQuadraticApproximation.dfduu = matrix_array_t(results.size(), matrix_t::Zero(u.size(), u.size()));

  pinocchioInterface_.computeAllJacobians(x);

  for (size_t i = 0; i < results.size(); ++i) {
    // Distance violation
    const hpp::fcl::DistanceResult& result = results[i];
    distanceQuadraticApproximation.f[i] = result.min_distance - minimumDistance_;

    // Jacobian calculation
    pinocchio::CollisionPair collisionPair = pinocchioGeometrySelfCollisions_.getGeometryModel().collisionPairs[i];
    const pinocchio::GeometryObject& geometryObject1 =
        pinocchioGeometrySelfCollisions_.getGeometryModel().geometryObjects[collisionPair.first];

    // We need to get the jacobian of the point on the first object; use the joint jacobian translated to the point
    const ocs2::Pose<scalar_t> joint1Pose = pinocchioInterface_.getJointPose(geometryObject1.parentJoint, x);
    const Eigen::Vector3d pt1Offset = result.nearest_points[0] - joint1Pose.position;
    const Eigen::MatrixXd joint1Jacobian = pinocchioInterface_.getJacobianOfJoint(geometryObject1.parentJoint);
    // Jacobians from pinocchio are given as
    // [ position jacobian ]
    // [ rotation jacobian ]
    const Eigen::MatrixXd pt1Jacobian = joint1Jacobian.topRows(3) - skewSymmetricMatrix(pt1Offset) * joint1Jacobian.bottomRows(3);

    const pinocchio::GeometryObject& geometryObject2 =
        pinocchioGeometrySelfCollisions_.getGeometryModel().geometryObjects[collisionPair.second];

    // We need to get the jacobian of the point on the second object; use the joint jacobian translated to the point
    const Pose<scalar_t> joint2Pose = pinocchioInterface_.getJointPose(geometryObject2.parentJoint, x);
    const Eigen::Vector3d pt2Offset = result.nearest_points[0] - joint2Pose.position;
    const Eigen::MatrixXd joint2Jacobian = pinocchioInterface_.getJacobianOfJoint(geometryObject2.parentJoint);
    const Eigen::MatrixXd pt2Jacobian = joint2Jacobian.topRows(3) - skewSymmetricMatrix(pt2Offset) * joint2Jacobian.bottomRows(3);

    // To get the (approximate) jacobian of the distance, get the difference between the two nearest point jacobians, then multiply by the
    // vector from point to point
    const Eigen::MatrixXd differenceJacobian = pt2Jacobian - pt1Jacobian;
    // TODO(perry): is there a way to calculate a correct jacobian for the case of distanceVector = 0?
    const Eigen::Vector3d distanceVector = result.min_distance > 0 ? (result.nearest_points[1] - result.nearest_points[0]).normalized()
                                                                   : (result.nearest_points[0] - result.nearest_points[1]).normalized();
    distanceQuadraticApproximation.dfdx.row(i) = distanceVector.transpose() * differenceJacobian;
  }

  ScalarFunctionQuadraticApproximation returnvalue =
      relaxedBarrierPenalty_.penaltyCostQuadraticApproximation(distanceQuadraticApproximation);

  return relaxedBarrierPenalty_.penaltyCostQuadraticApproximation(distanceQuadraticApproximation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation SelfCollisionCost::finalCostQuadraticApproximation(scalar_t t, const vector_t& x) {
  ScalarFunctionQuadraticApproximation approximation;
  approximation.f = 0.0;
  approximation.dfdx.setZero(9);
  approximation.dfdu.setZero(8);
  approximation.dfdxx.setZero(9, 9);
  approximation.dfdux.setZero(9, 8);
  approximation.dfduu.setZero(8, 8);
  return approximation;
}

}  // namespace ocs2
