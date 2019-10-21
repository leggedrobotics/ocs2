/*
 * AnymalCost.h
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#pragma once

#include <ocs2_switched_model_interface/cost/SwitchedModelCostBase.h>

namespace anymal {

class AnymalCost final : public switched_model::SwitchedModelCostBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = SwitchedModelCostBase;
  using Base::logic_rules_t;

  AnymalCost(std::shared_ptr<const logic_rules_t> logicRulesPtr, const state_matrix_t& Q, const input_matrix_t& R,
             const state_matrix_t& QFinal);

  AnymalCost(const AnymalCost& rhs);

  ~AnymalCost() override = default;
};

}  // end of namespace anymal
