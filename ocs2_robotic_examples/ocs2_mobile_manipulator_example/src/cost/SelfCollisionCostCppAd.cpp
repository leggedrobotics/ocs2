/*
 * SelfCollisionCostCppAd.cpp
 *
 *  Created on: 8 Sep 2020
 *      Author: perry
 */

#include <ocs2_mobile_manipulator_example/cost/SelfCollisionCostCppAd.h>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

#include <pinocchio/multibody/geometry.hpp>

namespace ocs2 {

SelfCollisionCostCppAd::SelfCollisionCostCppAd(ocs2::PinocchioInterface<double>& pinocchioInterface,
                                               const ocs2::PinocchioGeometryInterface& geometryInterfaceSelfCollision,
                                               scalar_t minimumDistance, scalar_t mu, scalar_t delta)
    : pinocchioInterface_(pinocchioInterface),
      pinocchioInterfaceAd_(ocs2::PinocchioInterface<double>::castToCppAd(pinocchioInterface)),
      pinocchioGeometrySelfCollisions_(geometryInterfaceSelfCollision),
      minimumDistance_(minimumDistance),
      relaxedBarrierPenalty_(mu, delta) {}

void SelfCollisionCostCppAd::initialize(const std::string& modelName, const std::string& modelFolder, bool recompileLibraries,
                                        bool verbose) {
  setADInterfaces(modelName, modelFolder);
  if (recompileLibraries) {
    createModels(verbose);
  } else {
    loadModelsIfAvailable(verbose);
  }
}

/** Evaluate the cost */
scalar_t SelfCollisionCostCppAd::cost(scalar_t t, const vector_t& x, const vector_t& u) {
  const std::vector<hpp::fcl::DistanceResult> results = pinocchioGeometrySelfCollisions_.computeDistances(x);

  vector_t violations = vector_t::Zero(results.size());
  for (size_t i = 0; i < results.size(); ++i) {
    const hpp::fcl::DistanceResult& result = results[i];
    violations[i] = result.min_distance - minimumDistance_;
  }

  return relaxedBarrierPenalty_.penaltyCost(violations);
}

/** Evaluate the final cost */
scalar_t SelfCollisionCostCppAd::finalCost(scalar_t t, const vector_t& x) {
  return 0;
}

/** Evaluate the cost quadratic approximation */
ScalarFunctionQuadraticApproximation SelfCollisionCostCppAd::costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  const std::vector<hpp::fcl::DistanceResult> results = pinocchioGeometrySelfCollisions_.computeDistances(x);

  vector_t points(results.size() * numberOfParamsPerResult_);

  for (size_t i = 0; i < results.size(); ++i) {
    std::pair<size_t, size_t> currentCollisionPair = pinocchioGeometrySelfCollisions_.getGeometryModel().collisionPairs[i];
    const hpp::fcl::DistanceResult& result = results[i];
    const pinocchio::GeometryObject& geometryObject1 =
        pinocchioGeometrySelfCollisions_.getGeometryModel().geometryObjects[currentCollisionPair.first];
    Pose<scalar_t> poseJoint1 = pinocchioInterface_.getJointPose(geometryObject1.parentJoint, x);
    Pose<scalar_t> inversePoseJoint1;
    inversePoseJoint1.position = poseJoint1.orientation.inverse() * -poseJoint1.position;
    inversePoseJoint1.orientation = poseJoint1.orientation.inverse();
    const pinocchio::GeometryObject& geometryObject2 =
        pinocchioGeometrySelfCollisions_.getGeometryModel().geometryObjects[currentCollisionPair.second];
    Pose<scalar_t> poseJoint2 = pinocchioInterface_.getJointPose(geometryObject2.parentJoint, x);
    Pose<scalar_t> inversePoseJoint2;
    inversePoseJoint2.position = poseJoint2.orientation.inverse() * -poseJoint2.position;
    inversePoseJoint2.orientation = poseJoint2.orientation.inverse();

    points.segment(i * numberOfParamsPerResult_, 3) = inversePoseJoint1.position + inversePoseJoint1.orientation * result.nearest_points[0];
    points.segment(i * numberOfParamsPerResult_ + 3, 3) =
        inversePoseJoint2.position + inversePoseJoint2.orientation * result.nearest_points[1];
    points[i * numberOfParamsPerResult_ + 6] = result.min_distance >= 0 ? 1.0 : -1.0;
  }

  VectorFunctionQuadraticApproximation distanceQuadraticApproximation;
  distanceQuadraticApproximation.f = cppAdInterface_->getFunctionValue(x, points);
  distanceQuadraticApproximation.dfdx = cppAdInterface_->getJacobian(x, points);
  distanceQuadraticApproximation.dfdu = matrix_t::Zero(results.size(), u.size());
  distanceQuadraticApproximation.dfdxx = matrix_array_t(results.size(), matrix_t::Zero(x.size(), x.size()));
  distanceQuadraticApproximation.dfdux = matrix_array_t(results.size(), matrix_t::Zero(x.size(), u.size()));
  distanceQuadraticApproximation.dfduu = matrix_array_t(results.size(), matrix_t::Zero(u.size(), u.size()));

  return relaxedBarrierPenalty_.penaltyCostQuadraticApproximation(distanceQuadraticApproximation);
}

/** Evaluate the final cost quadratic approximation */
ScalarFunctionQuadraticApproximation SelfCollisionCostCppAd::finalCostQuadraticApproximation(scalar_t t, const vector_t& x) {
  ScalarFunctionQuadraticApproximation approximation;
  approximation.f = 0.0;
  approximation.dfdx.setZero(9);
  approximation.dfdu.setZero(8);
  approximation.dfdxx.setZero(9, 9);
  approximation.dfdux.setZero(9, 8);
  approximation.dfduu.setZero(8, 8);
  return approximation;
}

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

void SelfCollisionCostCppAd::setADInterfaces(const std::string& modelName, const std::string& modelFolder) {
  using ad_interface = CppAdInterface;
  using ad_dynamic_vector_t = ad_interface::ad_vector_t;
  using ad_scalar_t = ad_interface::ad_scalar_t;

  const size_t numDistanceResults = this->pinocchioGeometrySelfCollisions_.getGeometryModel().collisionPairs.size();

  auto stateAndClosestPointsToDistance = [&, this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, -1> matrixResult = distanceCalculationAd(x, p);
    y = Eigen::Map<Eigen::Matrix<ad_scalar_t, -1, 1>>(matrixResult.data(), matrixResult.size());
  };
  cppAdInterface_.reset(new CppAdInterface(stateAndClosestPointsToDistance, mobile_manipulator::STATE_DIM,
                                           numDistanceResults * numberOfParamsPerResult_, modelName + "_intermediate", modelFolder));
}

void SelfCollisionCostCppAd::createModels(bool verbose) {
  cppAdInterface_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
}

void SelfCollisionCostCppAd::loadModelsIfAvailable(bool verbose) {
  cppAdInterface_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
}

} /* namespace ocs2 */
