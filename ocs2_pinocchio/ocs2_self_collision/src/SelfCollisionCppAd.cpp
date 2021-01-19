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

#include <ocs2_self_collision/SelfCollisionCppAd.h>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

#include <pinocchio/multibody/geometry.hpp>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionCppAd::SelfCollisionCppAd(scalar_t minimumDistance) : minimumDistance_(minimumDistance) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionCppAd::SelfCollisionCppAd(const SelfCollisionCppAd& rhs)
    : minimumDistance_(rhs.minimumDistance_),
      cppAdInterfaceDistanceCalculation_(new CppAdInterface(*rhs.cppAdInterfaceDistanceCalculation_)),
      cppAdInterfaceLinkPoints_(new CppAdInterface(*rhs.cppAdInterfaceLinkPoints_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SelfCollisionCppAd::initialize(const PinocchioInterface& pinocchioInterface, PinocchioGeometryInterface& pinocchioGeometryInterface,
                                    const std::string& modelName, const std::string& modelFolder, bool recompileLibraries, bool verbose) {
  PinocchioInterfaceCppAd pinocchioInterfaceAd = castToCppAd(pinocchioInterface);
  setADInterfaces(pinocchioInterfaceAd, pinocchioGeometryInterface, modelName, modelFolder);
  if (recompileLibraries) {
    createModels(verbose);
  } else {
    loadModelsIfAvailable(verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SelfCollisionCppAd::getValue(const PinocchioInterface& pinocchioInterface,
                                      const PinocchioGeometryInterface& pinocchioGeometryInterface) const {
  const std::vector<hpp::fcl::DistanceResult> distanceArray = pinocchioGeometryInterface.computeDistances(pinocchioInterface);

  vector_t violations = vector_t::Zero(distanceArray.size());
  for (size_t i = 0; i < distanceArray.size(); ++i) {
    violations[i] = distanceArray[i].min_distance - minimumDistance_;
  }

  return violations;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, matrix_t> SelfCollisionCppAd::getLinearApproximation(const PinocchioInterface& pinocchioInterface,
                                                                         const PinocchioGeometryInterface& pinocchioGeometryInterface,
                                                                         const vector_t& q) const {
  const std::vector<hpp::fcl::DistanceResult> distanceArray = pinocchioGeometryInterface.computeDistances(pinocchioInterface);

  vector_t pointsInWorldFrame(distanceArray.size() * numberOfParamsPerResult_);

  vector_t f(distanceArray.size());
  matrix_t dfdq(distanceArray.size(), q.size());
  for (size_t i = 0; i < distanceArray.size(); ++i) {
    pointsInWorldFrame.segment(i * numberOfParamsPerResult_, 3) = distanceArray[i].nearest_points[0];
    pointsInWorldFrame.segment(i * numberOfParamsPerResult_ + 3, 3) = distanceArray[i].nearest_points[1];
    pointsInWorldFrame[i * numberOfParamsPerResult_ + 6] = distanceArray[i].min_distance >= 0 ? 1.0 : -1.0;
  }

  vector_t pointsInLinkFrame = cppAdInterfaceLinkPoints_->getFunctionValue(q, pointsInWorldFrame);
  f = cppAdInterfaceDistanceCalculation_->getFunctionValue(q, pointsInLinkFrame);
  dfdq = cppAdInterfaceDistanceCalculation_->getJacobian(q, pointsInLinkFrame);

  return std::make_pair(f, dfdq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t SelfCollisionCppAd::computeLinkPointsAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd,
                                                    PinocchioGeometryInterface& pinocchioGeometryInterface, ad_vector_t state,
                                                    ad_vector_t points) {
  using Vector3 = Eigen::Matrix<ad_scalar_t, 3, 1>;
  using Quaternion = Eigen::Quaternion<ad_scalar_t>;

  pinocchioInterfaceAd.forwardKinematics(state);
  pinocchioInterfaceAd.updateGlobalPlacements();

  ad_vector_t pointsInLinkFrames = ad_vector_t::Zero(points.size());
  for (size_t i = 0; i < points.size() / numberOfParamsPerResult_; ++i) {
    const auto collisionPair = pinocchioGeometryInterface.getGeometryModel().collisionPairs[i];
    const pinocchio::GeometryObject& geometryObject1 = pinocchioGeometryInterface.getGeometryModel().geometryObjects[collisionPair.first];
    const auto joint1Position = pinocchioInterfaceAd.getJointPosition(geometryObject1.parentJoint);
    const auto joint1Orientation = pinocchioInterfaceAd.getJointOrientation(geometryObject1.parentJoint);
    const Vector3 joint1PositionInverse = joint1Orientation.conjugate() * -joint1Position;
    const Quaternion joint1OrientationInverse = joint1Orientation.conjugate();
    const pinocchio::GeometryObject& geometryObject2 = pinocchioGeometryInterface.getGeometryModel().geometryObjects[collisionPair.second];
    const auto joint2Position = pinocchioInterfaceAd.getJointPosition(geometryObject2.parentJoint);
    const auto joint2Orientation = pinocchioInterfaceAd.getJointOrientation(geometryObject2.parentJoint);
    const Vector3 joint2PositionInverse = joint2Orientation.conjugate() * -joint2Position;
    const Quaternion joint2OrientationInverse = joint2Orientation.conjugate();

    ad_vector_t point1 = points.segment(i * numberOfParamsPerResult_, 3);
    ad_vector_t point2 = points.segment(i * numberOfParamsPerResult_ + 3, 3);

    pointsInLinkFrames.segment(i * numberOfParamsPerResult_, 3) = joint1PositionInverse + joint1OrientationInverse * point1;
    pointsInLinkFrames.segment(i * numberOfParamsPerResult_ + 3, 3) = joint2PositionInverse + joint2OrientationInverse * point2;
    pointsInLinkFrames[i * numberOfParamsPerResult_ + 6] =
        CppAD::CondExpGt(points[i * numberOfParamsPerResult_ + 6], ad_scalar_t(0.0), ad_scalar_t(1.0), ad_scalar_t(-1.0));
  }
  return pointsInLinkFrames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t SelfCollisionCppAd::distanceCalculationAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd,
                                                      PinocchioGeometryInterface& pinocchioGeometryInterface, ad_vector_t state,
                                                      ad_vector_t points) {
  pinocchioInterfaceAd.forwardKinematics(state);
  pinocchioInterfaceAd.updateGlobalPlacements();

  ad_vector_t results = ad_vector_t::Zero(points.size() / numberOfParamsPerResult_);
  for (size_t i = 0; i < points.size() / numberOfParamsPerResult_; ++i) {
    const auto collisionPair = pinocchioGeometryInterface.getGeometryModel().collisionPairs[i];
    ad_vector_t point1 = points.segment(i * numberOfParamsPerResult_, 3);

    const pinocchio::GeometryObject& geometryObject1 = pinocchioGeometryInterface.getGeometryModel().geometryObjects[collisionPair.first];

    const auto joint1Position = pinocchioInterfaceAd.getJointPosition(geometryObject1.parentJoint);
    const auto joint1Orientation = pinocchioInterfaceAd.getJointOrientation(geometryObject1.parentJoint);
    ad_vector_t point1InWorld = joint1Position + joint1Orientation * point1;

    ad_vector_t point2 = points.segment(i * numberOfParamsPerResult_ + 3, 3);

    const pinocchio::GeometryObject& geometryObject2 = pinocchioGeometryInterface.getGeometryModel().geometryObjects[collisionPair.second];

    const auto joint2Position = pinocchioInterfaceAd.getJointPosition(geometryObject2.parentJoint);
    const auto joint2Orientation = pinocchioInterfaceAd.getJointOrientation(geometryObject2.parentJoint);

    ad_vector_t point2InWorld = joint2Position + joint2Orientation * point2;

    results[i] = points[i * numberOfParamsPerResult_ + 6] * (point2InWorld - point1InWorld).norm() - minimumDistance_;
  }
  return results;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SelfCollisionCppAd::setADInterfaces(PinocchioInterfaceCppAd& pinocchioInterfaceAd,
                                         PinocchioGeometryInterface& pinocchioGeometryInterface, const std::string& modelName,
                                         const std::string& modelFolder) {
  const size_t stateDim = pinocchioInterfaceAd.getModel().nq;
  const size_t numDistanceResults = pinocchioGeometryInterface.getGeometryModel().collisionPairs.size();

  auto stateAndClosestPointsToDistance = [&, this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, -1> matrixResult =
        distanceCalculationAd(pinocchioInterfaceAd, pinocchioGeometryInterface, x, p);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };
  cppAdInterfaceDistanceCalculation_.reset(new CppAdInterface(stateAndClosestPointsToDistance, stateDim,
                                                              numDistanceResults * numberOfParamsPerResult_,
                                                              modelName + "_distance_intermediate", modelFolder));

  auto stateAndClosestPointsToLinkFrame = [&, this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, -1> matrixResult =
        computeLinkPointsAd(pinocchioInterfaceAd, pinocchioGeometryInterface, x, p);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };
  cppAdInterfaceLinkPoints_.reset(new CppAdInterface(stateAndClosestPointsToLinkFrame, stateDim,
                                                     numDistanceResults * numberOfParamsPerResult_, modelName + "_links_intermediate",
                                                     modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SelfCollisionCppAd::createModels(bool verbose) {
  cppAdInterfaceDistanceCalculation_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  cppAdInterfaceLinkPoints_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SelfCollisionCppAd::loadModelsIfAvailable(bool verbose) {
  cppAdInterfaceDistanceCalculation_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  cppAdInterfaceLinkPoints_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
}

} /* namespace ocs2 */
