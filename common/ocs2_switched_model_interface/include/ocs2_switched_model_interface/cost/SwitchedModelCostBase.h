/*
 * SwitchedModelCostBase.h
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#pragma once

#include <ocs2_core/cost/CostFunctionBase.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/ModelSettings.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/cost/FootPlacementCost.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

namespace switched_model {

class SwitchedModelCostBase : public ocs2::CostFunctionBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using ad_interface_t = ocs2::CppAdInterface;
  using ad_scalar_t = ocs2::CppAdInterface::ad_scalar_t;
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  using com_model_t = ComModelBase<scalar_t>;

  //! Constructor
  //! Constructor
  SwitchedModelCostBase(const com_model_t& comModel, const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel,
                        const SwitchedModelModeScheduleManager& modeScheduleManager, const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                        const state_matrix_t& Q, const input_matrix_t& R, ModelSettings options = ModelSettings());

  //! Destructor
  ~SwitchedModelCostBase() override = default;

  //! clone SwitchedModelCostBase class.
  SwitchedModelCostBase* clone() const override;

  scalar_t cost(scalar_t t, const vector_t& x, const vector_t& u) override;
  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) override;

  scalar_t finalCost(scalar_t t, const vector_t& x) override;
  ScalarFunctionQuadraticApproximation finalCostQuadraticApproximation(scalar_t t, const vector_t& x) override;

 private:
  //! Copy constructor
  SwitchedModelCostBase(const SwitchedModelCostBase& rhs);

  void update(scalar_t t, const vector_t& x, const vector_t& u);

  void inputFromContactFlags(const contact_flag_t& contactFlags, const state_vector_t& nominalState, vector_t& inputs) const;

  std::unique_ptr<FootPlacementCost> footPlacementCost_;
  std::unique_ptr<com_model_t> comModelPtr_;

  const SwitchedModelModeScheduleManager* modeScheduleManagerPtr_;
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;

  // Quadratic cost terms
  state_matrix_t Q_;
  input_matrix_t R_;
};

}  // end of namespace switched_model
