/*
 * SwitchedModelCostBase.h
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#pragma once

#include <ocs2_core/cost/QuadraticCostFunction.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM = 12 + JOINT_COORD_SIZE, size_t INPUT_DIM = 12 + JOINT_COORD_SIZE>
class SwitchedModelCostBase : public ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM> BASE;
  using typename BASE::input_matrix_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;

  using logic_rules_t = SwitchedModelPlannerLogicRules<JOINT_COORD_SIZE, double>;
  using contact_flag_t = typename SwitchedModel<JOINT_COORD_SIZE>::contact_flag_t;

  using com_model_t = ComModelBase<JOINT_COORD_SIZE>;

  //! Constructor
  SwitchedModelCostBase(const com_model_t& comModel, std::shared_ptr<const logic_rules_t> logicRulesPtr, const state_matrix_t& Q,
                        const input_matrix_t& R, const state_matrix_t& QFinal);

  //! Copy constructor
  SwitchedModelCostBase(const SwitchedModelCostBase& rhs);

  //! Destructor
  ~SwitchedModelCostBase() override = default;

  //! clone SwitchedModelCostBase class.
  SwitchedModelCostBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>* clone() const override;

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;

 private:
  void inputFromContactFlags(contact_flag_t contactFlags, input_vector_t& inputs);

  std::unique_ptr<com_model_t> comModelPtr_;

  std::shared_ptr<const logic_rules_t> logicRulesPtr_;
};

}  // end of namespace switched_model

#include "implementation/SwitchedModelCostBase.h"
