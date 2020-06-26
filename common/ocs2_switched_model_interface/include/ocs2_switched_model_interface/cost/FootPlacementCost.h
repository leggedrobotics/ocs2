//
// Created by rgrandia on 26.06.20.
//

#pragma once

#include <ocs2_core/Dimensions.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/foot_planner/FootPhase.h"
#include "ocs2_switched_model_interface/terrain/ConvexTerrain.h"

namespace switched_model {

struct FootPlacementCostParameters {
  scalar_t mu = 1.0;      // magnitude scaling
  scalar_t delta = 0.01;  // [m] distance from constraint boundary where the barrier becomes quadratic.
};

class FootPlacementCost {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using ad_interface_t = ocs2::CppAdInterface;
  using ad_scalar_t = typename ad_interface_t::ad_scalar_t;
  using ad_dynamic_vector_t = typename ad_interface_t::ad_dynamic_vector_t;
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  using DIMENSIONS = ocs2::Dimensions<STATE_DIM, INPUT_DIM>;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;

  using analytic_jacobian_t = typename Eigen::Matrix<scalar_t, 3, STATE_DIM>;

  FootPlacementCost(FootPlacementCostParameters settings, const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel,
                    bool generateModels);

  FootPlacementCost* clone() const;

  void setStateAndConstraint(const state_vector_t& x, const feet_array_t<FootTangentialConstraintMatrix*>& constraints);

  scalar_t getCostValue();
  state_vector_t getCostDerivativeState();
  state_matrix_t getCostSecondDerivativeState();

 private:
  FootPlacementCost(const FootPlacementCost& rhs);
  void initAdModels(bool generateModels, bool verbose = true);
  static void adfunc(const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel, const ad_dynamic_vector_t& state,
                     ad_dynamic_vector_t& o_feetPositions);
  void updateJacobians();
  void updateConstraintValues();

  scalar_t getPenaltyFunctionValue(scalar_t h, const FootPlacementCostParameters& config) const;
  scalar_t getPenaltyFunctionDerivative(scalar_t h, const FootPlacementCostParameters& config) const;
  scalar_t getPenaltyFunctionSecondDerivative(scalar_t h, const FootPlacementCostParameters& config) const;

  FootPlacementCostParameters settings_;
  state_vector_t x_;
  feet_array_t<FootTangentialConstraintMatrix*> constraints_;

  bool constraintValuesUpdated_;
  feet_array_t<dynamic_vector_t> constraintValues_;

  bool feetJacobiansUpdated_;
  feet_array_t<analytic_jacobian_t> feetJacobiansInOrigin_;

  std::unique_ptr<ad_interface_t> adInterface_;
};

}  // namespace switched_model
