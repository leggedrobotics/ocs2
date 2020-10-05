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

/*
 * SelfCollisionCostCppAd.cpp
 *
 *  Created on: 8 Sep 2020
 *      Author: perry
 */

#include <ocs2_pinocchio/cost/SelfCollisionCostCppAd.h>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

#include <pinocchio/multibody/geometry.hpp>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionCostCppAd::SelfCollisionCostCppAd(PinocchioInterface pinocchioInterface,
                                               PinocchioGeometryInterface geometryInterfaceSelfCollision, scalar_t minimumDistance,
                                               scalar_t mu, scalar_t delta)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      pinocchioInterfaceAd_(castToCppAd(pinocchioInterface_)),
      pinocchioGeometrySelfCollisions_(geometryInterfaceSelfCollision),
      minimumDistance_(minimumDistance),
      relaxedBarrierPenalty_(mu, delta) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SelfCollisionCostCppAd::SelfCollisionCostCppAd(const SelfCollisionCostCppAd& rhs)
    : pinocchioInterface_(rhs.pinocchioInterface_),
      pinocchioInterfaceAd_(castToCppAd(pinocchioInterface_)),
      pinocchioGeometrySelfCollisions_(rhs.pinocchioGeometrySelfCollisions_),
      minimumDistance_(rhs.minimumDistance_),
      relaxedBarrierPenalty_(rhs.relaxedBarrierPenalty_),
      cppAdInterfaceDistanceCalculation_(new CppAdInterface(*rhs.cppAdInterfaceDistanceCalculation_)),
      cppAdInterfaceLinkPoints_(new CppAdInterface(*rhs.cppAdInterfaceLinkPoints_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SelfCollisionCostCppAd::initialize(const std::string& modelName, const std::string& modelFolder, bool recompileLibraries,
                                        bool verbose) {
  setADInterfaces(modelName, modelFolder);
  if (recompileLibraries) {
    createModels(verbose);
  } else {
    loadModelsIfAvailable(verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SelfCollisionCostCppAd::cost(scalar_t t, const vector_t& x, const vector_t& u) {
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
scalar_t SelfCollisionCostCppAd::finalCost(scalar_t t, const vector_t& x) {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation SelfCollisionCostCppAd::costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  const std::vector<hpp::fcl::DistanceResult> results = pinocchioGeometrySelfCollisions_.computeDistances(x);

  vector_t pointsInWorldFrame(results.size() * numberOfParamsPerResult_);

  VectorFunctionQuadraticApproximation distanceQuadraticApproximation;
  distanceQuadraticApproximation.f.resize(results.size());
  for (size_t i = 0; i < results.size(); ++i) {
    const hpp::fcl::DistanceResult& result = results[i];
    pointsInWorldFrame.segment(i * numberOfParamsPerResult_, 3) = result.nearest_points[0];
    pointsInWorldFrame.segment(i * numberOfParamsPerResult_ + 3, 3) = result.nearest_points[1];
    pointsInWorldFrame[i * numberOfParamsPerResult_ + 6] = result.min_distance >= 0 ? 1.0 : -1.0;
  }

  vector_t pointsInLinkFrame = cppAdInterfaceLinkPoints_->getFunctionValue(x, pointsInWorldFrame);
  distanceQuadraticApproximation.f = cppAdInterfaceDistanceCalculation_->getFunctionValue(x, pointsInLinkFrame);
  distanceQuadraticApproximation.dfdx = cppAdInterfaceDistanceCalculation_->getJacobian(x, pointsInLinkFrame);
  distanceQuadraticApproximation.dfdu = matrix_t::Zero(results.size(), u.size());
  distanceQuadraticApproximation.dfdxx = matrix_array_t(results.size(), matrix_t::Zero(x.size(), x.size()));
  distanceQuadraticApproximation.dfdux = matrix_array_t(results.size(), matrix_t::Zero(x.size(), u.size()));
  distanceQuadraticApproximation.dfduu = matrix_array_t(results.size(), matrix_t::Zero(u.size(), u.size()));

  return relaxedBarrierPenalty_.penaltyCostQuadraticApproximation(distanceQuadraticApproximation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation SelfCollisionCostCppAd::finalCostQuadraticApproximation(scalar_t t, const vector_t& x) {
  return ScalarFunctionQuadraticApproximation::Zero(x.rows(), 0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t SelfCollisionCostCppAd::computeLinkPointsAd(ad_vector_t state, ad_vector_t points) {
  ad_vector_t pointsInLinkFrames = ad_vector_t::Zero(points.size());
  for (size_t i = 0; i < points.size() / numberOfParamsPerResult_; ++i) {
    std::pair<size_t, size_t> currentCollisionPair = pinocchioGeometrySelfCollisions_.getGeometryModel().collisionPairs[i];
    const pinocchio::GeometryObject& geometryObject1 =
        pinocchioGeometrySelfCollisions_.getGeometryModel().geometryObjects[currentCollisionPair.first];
    Pose<ad_scalar_t> poseJoint1 = pinocchioInterfaceAd_.getJointPose(geometryObject1.parentJoint, state);
    Pose<ad_scalar_t> inversePoseJoint1;
    inversePoseJoint1.position = poseJoint1.orientation.conjugate() * -poseJoint1.position;
    inversePoseJoint1.orientation = poseJoint1.orientation.conjugate();
    const pinocchio::GeometryObject& geometryObject2 =
        pinocchioGeometrySelfCollisions_.getGeometryModel().geometryObjects[currentCollisionPair.second];
    Pose<ad_scalar_t> poseJoint2 = pinocchioInterfaceAd_.getJointPose(geometryObject2.parentJoint, state);
    Pose<ad_scalar_t> inversePoseJoint2;
    inversePoseJoint2.position = poseJoint2.orientation.conjugate() * -poseJoint2.position;
    inversePoseJoint2.orientation = poseJoint2.orientation.conjugate();

    ad_vector_t point1 = points.segment(i * numberOfParamsPerResult_, 3);
    ad_vector_t point2 = points.segment(i * numberOfParamsPerResult_ + 3, 3);

    pointsInLinkFrames.segment(i * numberOfParamsPerResult_, 3) = inversePoseJoint1.position + inversePoseJoint1.orientation * point1;
    pointsInLinkFrames.segment(i * numberOfParamsPerResult_ + 3, 3) = inversePoseJoint2.position + inversePoseJoint2.orientation * point2;
    pointsInLinkFrames[i * numberOfParamsPerResult_ + 6] =
        CppAD::CondExpGt(points[i * numberOfParamsPerResult_ + 6], ad_scalar_t(0.0), ad_scalar_t(1.0), ad_scalar_t(-1.0));
  }
  return pointsInLinkFrames;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t SelfCollisionCostCppAd::distanceCalculationAd(ad_vector_t state, ad_vector_t points) {
  ad_vector_t results = ad_vector_t::Zero(points.size() / numberOfParamsPerResult_);
  for (size_t i = 0; i < points.size() / numberOfParamsPerResult_; ++i) {
    std::pair<size_t, size_t> currentCollisionPair = pinocchioGeometrySelfCollisions_.getGeometryModel().collisionPairs[i];
    ad_vector_t point1 = points.segment(i * numberOfParamsPerResult_, 3);

    const pinocchio::GeometryObject& geometryObject1 =
        pinocchioGeometrySelfCollisions_.getGeometryModel().geometryObjects[currentCollisionPair.first];
    Pose<ad_scalar_t> poseJoint1 = pinocchioInterfaceAd_.getJointPose(geometryObject1.parentJoint, state);

    ad_vector_t point1InWorld = poseJoint1.position + poseJoint1.orientation * point1;

    ad_vector_t point2 = points.segment(i * numberOfParamsPerResult_ + 3, 3);

    const pinocchio::GeometryObject& geometryObject2 =
        pinocchioGeometrySelfCollisions_.getGeometryModel().geometryObjects[currentCollisionPair.second];
    Pose<ad_scalar_t> poseJoint2 = pinocchioInterfaceAd_.getJointPose(geometryObject2.parentJoint, state);

    ad_vector_t point2InWorld = poseJoint2.position + poseJoint2.orientation * point2;

    results[i] = points[i * numberOfParamsPerResult_ + 6] * (point2InWorld - point1InWorld).norm() - minimumDistance_;
  }
  return results;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SelfCollisionCostCppAd::setADInterfaces(const std::string& modelName, const std::string& modelFolder) {
  const size_t stateDim = pinocchioInterface_.getModel().nq;
  const size_t numDistanceResults = this->pinocchioGeometrySelfCollisions_.getGeometryModel().collisionPairs.size();

  auto stateAndClosestPointsToDistance = [&, this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, -1> matrixResult = distanceCalculationAd(x, p);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };
  cppAdInterfaceDistanceCalculation_.reset(new CppAdInterface(stateAndClosestPointsToDistance, stateDim,
                                                              numDistanceResults * numberOfParamsPerResult_,
                                                              modelName + "_distance_intermediate", modelFolder));

  auto stateAndClosestPointsToLinkFrame = [&, this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, -1> matrixResult = computeLinkPointsAd(x, p);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };
  cppAdInterfaceLinkPoints_.reset(new CppAdInterface(stateAndClosestPointsToLinkFrame, stateDim,
                                                     numDistanceResults * numberOfParamsPerResult_, modelName + "_links_intermediate",
                                                     modelFolder));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SelfCollisionCostCppAd::createModels(bool verbose) {
  cppAdInterfaceDistanceCalculation_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  cppAdInterfaceLinkPoints_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SelfCollisionCostCppAd::loadModelsIfAvailable(bool verbose) {
  cppAdInterfaceDistanceCalculation_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  cppAdInterfaceLinkPoints_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
}

} /* namespace ocs2 */
