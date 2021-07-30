//
// Created by rgrandia on 26.06.20.
//

#pragma once

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/cost/LinearStateInequalitySoftconstraint.h"
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
  using com_model_t = ComModelBase<scalar_t>;
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using kinematic_model_t = KinematicsModelBase<scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  FootPlacementCost(ocs2::RelaxedBarrierPenalty::Config settings, ocs2::RelaxedBarrierPenalty::Config sdfSettings,
                    const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                    const ad_com_model_t& adComModel, bool generateModels);

  FootPlacementCost* clone() const;

  void setConstraints(const feet_array_t<const FootTangentialConstraintMatrix*>& constraints,
                      const feet_array_t<SignedDistanceConstraint>& sdfConstraints, const SignedDistanceField* sdfPtr);

  scalar_t getValue(const vector_t& x);
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(const vector_t& x);

 private:
  FootPlacementCost(const FootPlacementCost& rhs);
  static void adfunc(const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel, const ad_vector_t& state,
                     ad_vector_t& targetsInOriginFrame);

  void updatePositions(const vector_t& x);
  void updateJacobians(const vector_t& x);
  void updateConstraintValues(const vector_t& x);

  std::unique_ptr<ocs2::RelaxedBarrierPenalty> polygonPenalty_;
  std::unique_ptr<ocs2::RelaxedBarrierPenalty> sdfPenalty_;

  std::vector<vector_t> targetPositions_;
  std::vector<matrix_t> targetJacobians_;
  std::vector<std::vector<LinearStateInequalitySoftConstraint>> constraintsPerTarget_;
  std::vector<scalar_t> targetRadii_;

  feet_array_t<const FootTangentialConstraintMatrix*> constraints_;
  feet_array_t<SignedDistanceConstraint> sdfConstraints_;
  const SignedDistanceField* sdfPtr_;

  std::unique_ptr<ad_interface_t> adInterface_;
};

}  // namespace switched_model
