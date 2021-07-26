//
// Created by rgrandia on 26.06.20.
//

#pragma once

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/foot_planner/FootPhase.h"
#include "ocs2_switched_model_interface/terrain/ConvexTerrain.h"

#include <ocs2_core/soft_constraint/penalties/RelaxedBarrierPenalty.h>

namespace switched_model {

class FootPlacementCost {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using ad_interface_t = ocs2::CppAdInterface;
  using ad_scalar_t = ocs2::CppAdInterface::ad_scalar_t;
  using ad_vector_t = ocs2::CppAdInterface::ad_vector_t;
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  using analytic_jacobian_t = typename Eigen::Matrix<scalar_t, 3, STATE_DIM>;

  FootPlacementCost(ocs2::RelaxedBarrierPenalty::Config settings, ocs2::RelaxedBarrierPenalty::Config sdfSettings,
                    const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel, bool generateModels);

  FootPlacementCost* clone() const;

  void setConstraints(const feet_array_t<const FootTangentialConstraintMatrix*>& constraints,
                      const feet_array_t<SignedDistanceConstraint>& sdfConstraints);

  scalar_t getValue(const vector_t& x);
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(const vector_t& x);

 private:
  FootPlacementCost(const FootPlacementCost& rhs);
  static void adfunc(const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel, const ad_vector_t& state,
                     ad_vector_t& o_feetPositions);

  scalar_t getCostValue() const;
  vector_t getCostDerivativeState() const;
  matrix_t getCostSecondDerivativeState() const;
  void updateJacobians(const vector_t& x);
  void updateConstraintValues(const vector_t& x);

  std::unique_ptr<ocs2::RelaxedBarrierPenalty> polygonPenalty_;
  std::unique_ptr<ocs2::RelaxedBarrierPenalty> sdfPenalty_;

  feet_array_t<const FootTangentialConstraintMatrix*> constraints_;
  feet_array_t<SignedDistanceConstraint> sdfConstraints_;

  feet_array_t<vector_t> constraintValues_;
  feet_array_t<std::pair<scalar_t, vector3_t>> sdfValues_;

  feet_array_t<analytic_jacobian_t> feetJacobiansInOrigin_;

  std::unique_ptr<ad_interface_t> adInterface_;
};

}  // namespace switched_model
