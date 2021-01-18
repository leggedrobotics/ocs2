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

#include <ocs2_self_collision/SelfCollision.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

#include <pinocchio/multibody/geometry.hpp>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollision::SelfCollision(ocs2::PinocchioGeometryInterface geometryInterfaceSelfCollision, scalar_t minimumDistance)
    : pinocchioGeometrySelfCollisions_(std::move(geometryInterfaceSelfCollision)), minimumDistance_(minimumDistance) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollision::SelfCollision(const SelfCollision& rhs)
    : pinocchioGeometrySelfCollisions_(rhs.pinocchioGeometrySelfCollisions_), minimumDistance_(rhs.minimumDistance_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SelfCollision::getValue(PinocchioInterface& pinocchioInterface, const vector_t& q) const {
  PinocchioGeometryInterface& pinocchioGeometrySelfCollisions = pinocchioGeometrySelfCollisions_;
  const std::vector<hpp::fcl::DistanceResult> distanceArray = pinocchioGeometrySelfCollisions.computeDistances(q);

  vector_t violations = vector_t::Zero(distanceArray.size());
  for (size_t i = 0; i < distanceArray.size(); ++i) {
    violations[i] = distanceArray[i].min_distance - minimumDistance_;
  }

  return violations;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, matrix_t> SelfCollision::getLinearApproximation(PinocchioInterface& pinocchioInterface, const vector_t& q) const {
  using Vector3 = Eigen::Matrix<scalar_t, 3, 1>;

  PinocchioGeometryInterface& pinocchioGeometrySelfCollisions = pinocchioGeometrySelfCollisions_;
  const std::vector<hpp::fcl::DistanceResult> distanceArray = pinocchioGeometrySelfCollisions.computeDistances(q);

  vector_t f(distanceArray.size());
  matrix_t dfdq(distanceArray.size(), q.size());

  pinocchioInterface.forwardKinematics(q);
  pinocchioInterface.updateGlobalPlacements();
  pinocchioInterface.computeJointJacobians(q);

  for (size_t i = 0; i < distanceArray.size(); ++i) {
    // Distance violation
    f[i] = distanceArray[i].min_distance - minimumDistance_;

    // Jacobian calculation
    pinocchio::CollisionPair collisionPair = pinocchioGeometrySelfCollisions_.getGeometryModel().collisionPairs[i];
    const pinocchio::GeometryObject& geometryObject1 =
        pinocchioGeometrySelfCollisions_.getGeometryModel().geometryObjects[collisionPair.first];

    // We need to get the jacobian of the point on the first object; use the joint jacobian translated to the point
    const Vector3 joint1Position = pinocchioInterface.getJointPosition(geometryObject1.parentJoint);
    const Vector3 pt1Offset = distanceArray[i].nearest_points[0] - joint1Position;
    const matrix_t joint1Jacobian = pinocchioInterface.getJointJacobian(geometryObject1.parentJoint);
    // Jacobians from pinocchio are given as
    // [ position jacobian ]
    // [ rotation jacobian ]
    const matrix_t pt1Jacobian = joint1Jacobian.topRows(3) - skewSymmetricMatrix(pt1Offset) * joint1Jacobian.bottomRows(3);

    const pinocchio::GeometryObject& geometryObject2 =
        pinocchioGeometrySelfCollisions_.getGeometryModel().geometryObjects[collisionPair.second];

    // We need to get the jacobian of the point on the second object; use the joint jacobian translated to the point
    const Vector3 joint2Position = pinocchioInterface.getJointPosition(geometryObject2.parentJoint);
    const Vector3 pt2Offset = distanceArray[i].nearest_points[1] - joint2Position;
    const matrix_t joint2Jacobian = pinocchioInterface.getJointJacobian(geometryObject2.parentJoint);
    const matrix_t pt2Jacobian = joint2Jacobian.topRows(3) - skewSymmetricMatrix(pt2Offset) * joint2Jacobian.bottomRows(3);

    // To get the (approximate) jacobian of the distance, get the difference between the two nearest point jacobians, then multiply by the
    // vector from point to point
    const matrix_t differenceJacobian = pt2Jacobian - pt1Jacobian;
    // TODO(perry): is there a way to calculate a correct jacobian for the case of distanceVector = 0?
    const Vector3 distanceVector = distanceArray[i].min_distance > 0
                                       ? (distanceArray[i].nearest_points[1] - distanceArray[i].nearest_points[0]).normalized()
                                       : (distanceArray[i].nearest_points[0] - distanceArray[i].nearest_points[1]).normalized();
    dfdq.row(i) = distanceVector.transpose() * differenceJacobian;
  }

  return std::make_pair(f, dfdq);
}

}  // namespace ocs2
