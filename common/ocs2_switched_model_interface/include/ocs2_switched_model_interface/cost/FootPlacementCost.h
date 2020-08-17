//
// Created by rgrandia on 26.06.20.
//

#pragma once

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/foot_planner/FootPhase.h"
#include "ocs2_switched_model_interface/terrain/ConvexTerrain.h"

namespace switched_model {

struct FootPlacementCostParameters {
  scalar_t mu = 0.1;      // magnitude scaling
  scalar_t delta = 0.01;  // [m] distance from constraint boundary where the barrier becomes quadratic.
};

class FootPlacementCost {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using ad_interface_t = ocs2::CppAdInterface;
  using ad_scalar_t = ocs2::CppAdInterface::ad_scalar_t;
  using ad_vector_t = ocs2::CppAdInterface::ad_vector_t;
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  using analytic_jacobian_t = typename Eigen::Matrix<scalar_t, 3, STATE_DIM>;

  FootPlacementCost(FootPlacementCostParameters settings, const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel,
                    bool generateModels);

  FootPlacementCost* clone() const;

  void setStateAndConstraint(const vector_t& x, const feet_array_t<const FootTangentialConstraintMatrix*>& constraints);

  scalar_t getCostValue();
  vector_t getCostDerivativeState();
  matrix_t getCostSecondDerivativeState();

 private:
  FootPlacementCost(const FootPlacementCost& rhs);
  void initAdModels(bool generateModels, bool verbose = true);
  static void adfunc(const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel, const ad_vector_t& state,
                     ad_vector_t& o_feetPositions);
  void updateJacobians();
  void updateConstraintValues();

  static scalar_t getPenaltyFunctionValue(scalar_t h, const FootPlacementCostParameters& config);
  static scalar_t getPenaltyFunctionDerivative(scalar_t h, const FootPlacementCostParameters& config);
  static scalar_t getPenaltyFunctionSecondDerivative(scalar_t h, const FootPlacementCostParameters& config);

  FootPlacementCostParameters settings_;
  state_vector_t x_;
  feet_array_t<const FootTangentialConstraintMatrix*> constraints_;

  bool constraintValuesUpdated_;
  feet_array_t<vector_t> constraintValues_;

  bool feetJacobiansUpdated_;
  feet_array_t<analytic_jacobian_t> feetJacobiansInOrigin_;

  std::unique_ptr<ad_interface_t> adInterface_;
};

}  // namespace switched_model
