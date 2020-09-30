/*
 * SelfCollisionCostCppAd.h
 *
 *  Created on: 8 Sep 2020
 *      Author: perry
 */

#pragma once

#include <ocs2_mobile_manipulator_example/definitions.h>
#include <ocs2_pinocchio/PinocchioInterface.h>
#include <ocs2_pinocchio/PinocchioGeometryInterface.hpp>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/constraint/RelaxedBarrierPenalty.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/cost/QuadraticGaussNewtonCostBaseAD.h>

namespace ocs2 {

using ad_vector_t = mobile_manipulator::ad_vector_t;

class SelfCollisionCostCppAd : public CostFunctionBase {
 public:
  //  SelfCollisionCostCppAd(PinocchioInterface<scalar_t> pinocchioInterface, );
  SelfCollisionCostCppAd(ocs2::PinocchioInterface<scalar_t> pinocchioInterface,
                         ocs2::PinocchioGeometryInterface geometryInterfaceSelfCollision, scalar_t minimumDistance, scalar_t mu,
                         scalar_t delta);
  ~SelfCollisionCostCppAd() override = default;
  SelfCollisionCostCppAd(const SelfCollisionCostCppAd& rhs);

  SelfCollisionCostCppAd* clone() const override { return new SelfCollisionCostCppAd(*this); }

  void initialize(const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true,
                  bool verbose = true);

  /** Evaluate the cost */
  scalar_t cost(scalar_t t, const vector_t& x, const vector_t& u) override;

  /** Evaluate the final cost */
  scalar_t finalCost(scalar_t t, const vector_t& x) override;

  /** Evaluate the cost quadratic approximation */
  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) override;

  /** Evaluate the final cost quadratic approximation */
  ScalarFunctionQuadraticApproximation finalCostQuadraticApproximation(scalar_t t, const vector_t& x) override;

  Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, 1> getCollisionObjectJointPoses();

 private:
  void setADInterfaces(const std::string& modelName, const std::string& modelFolder);
  void createModels(bool verbose);
  void loadModelsIfAvailable(bool verbose);

  // Number of params per result = 3 + 3 + 1 (nearest point 1, nearest point 2, sign indicator)
  const size_t numberOfParamsPerResult_ = 7;

  // From the current state of the robot, and the closest points in world frame, compute the positions of the points in link frame
  // In this case : size of state = stateDim, size of points = 3*2*number of collision pairs + 1 (for sign indicator)
  // Returns a vector that is of length |3*2*number of collision pairs + 1 (for sign indicator)|
  ad_vector_t computeLinkPointsAd(ad_vector_t state, ad_vector_t points);
  // From the current state of the robot, and the closest points in link frames, calculate the distances wrt state
  // In this case : size of state = stateDim, size of points = 3*2*number of collision pairs + 1 (for sign indicator)
  // Returns a vector that is of length |collisionPairs|
  ad_vector_t distanceCalculationAd(ad_vector_t state, ad_vector_t points);

  std::shared_ptr<ocs2::CppAdInterface> cppAdInterfaceDistanceCalculation_;
  std::shared_ptr<ocs2::CppAdInterface> cppAdInterfaceLinkPoints_;

  ocs2::PinocchioInterface<scalar_t> pinocchioInterface_;
  ocs2::PinocchioInterface<ad_scalar_t> pinocchioInterfaceAd_;
  ocs2::PinocchioGeometryInterface pinocchioGeometrySelfCollisions_;

  scalar_t minimumDistance_;

  const ocs2::RelaxedBarrierPenalty relaxedBarrierPenalty_;
};

} /* namespace ocs2 */
