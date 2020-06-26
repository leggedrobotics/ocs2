/*
 * SwitchedModelCostBase.h
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#pragma once

#include "ocs2_core/cost/CostFunctionBase.h"

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/cost/FootPlacementCost.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

namespace switched_model {

class SwitchedModelCostBase : public ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using ad_interface_t = ocs2::CppAdInterface;
  using ad_scalar_t = typename ad_interface_t::ad_scalar_t;
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  using BASE = ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM>;
  using typename BASE::dynamic_vector_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_vector_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;

  using com_model_t = ComModelBase<scalar_t>;

  //! Constructor
  SwitchedModelCostBase(const com_model_t& comModel, const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel,
                        std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr, const state_matrix_t& Q,
                        const input_matrix_t& R, const state_matrix_t& QFinal, bool generateModels);

  //! Copy constructor
  SwitchedModelCostBase(const SwitchedModelCostBase& rhs);

  //! Destructor
  ~SwitchedModelCostBase() override = default;

  //! clone SwitchedModelCostBase class.
  SwitchedModelCostBase* clone() const override;

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;

  void getIntermediateCost(scalar_t& L) override;
  void getIntermediateCostDerivativeState(state_vector_t& dLdx) override;
  void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) override;
  void getIntermediateCostDerivativeInput(input_vector_t& dLdu) override;
  void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) override;
  void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdux) override;

  void getTerminalCost(scalar_t& cost) override;
  void getTerminalCostDerivativeState(state_vector_t& dPhidx) override;
  void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) override;

 private:
  void inputFromContactFlags(const contact_flag_t& contactFlags, const state_vector_t& nominalState, dynamic_vector_t& inputs);

  std::unique_ptr<FootPlacementCost> footPlacementCost_;
  std::unique_ptr<com_model_t> comModelPtr_;

  std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr_;

  // Quadratic cost terms
  state_matrix_t Q_;
  input_matrix_t R_;
  state_matrix_t QFinal_;
  state_vector_t xIntermediateDeviation_;
  input_vector_t uIntermediateDeviation_;
  state_vector_t xNominalFinal_;
};

}  // end of namespace switched_model
