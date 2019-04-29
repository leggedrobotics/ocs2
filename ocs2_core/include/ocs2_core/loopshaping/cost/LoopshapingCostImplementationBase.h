//
// Created by rgrandia on 29.04.19.
//

#ifndef OCS2_LOOPSHAPINGCOSTIMPLEMENTATIONBASE_H
#define OCS2_LOOPSHAPINGCOSTIMPLEMENTATIONBASE_H

#include "ocs2_core/Dimensions.h"

namespace ocs2 {

template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
    size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
    size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
    class LOGIC_RULES_T=NullLogicRules>
class LoopshapingCostImplementationBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using FULL_DIMENSIONS = ocs2::Dimensions<FULL_STATE_DIM, FULL_INPUT_DIM>;
  using scalar_t = typename FULL_DIMENSIONS::scalar_t;
  using state_vector_t = typename FULL_DIMENSIONS::state_vector_t;
  using input_vector_t = typename FULL_DIMENSIONS::input_vector_t;
  using state_matrix_t = typename FULL_DIMENSIONS::state_matrix_t;
  using input_matrix_t = typename FULL_DIMENSIONS::input_matrix_t;
  using input_state_matrix_t = typename FULL_DIMENSIONS::input_state_matrix_t;

  using SYSTEM_DIMENSIONS = ocs2::Dimensions<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM>;
  using system_state_vector_t = typename SYSTEM_DIMENSIONS::state_vector_t;
  using system_input_vector_t = typename SYSTEM_DIMENSIONS::input_vector_t;
  using system_state_matrix_t = typename SYSTEM_DIMENSIONS::state_matrix_t;
  using system_input_matrix_t = typename SYSTEM_DIMENSIONS::input_matrix_t;
  using system_input_state_matrix_t = typename SYSTEM_DIMENSIONS::input_state_matrix_t;

  using FILTER_DIMENSIONS = ocs2::Dimensions<FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using filter_state_vector_t = typename FILTER_DIMENSIONS::state_vector_t;
  using filter_input_vector_t = typename FILTER_DIMENSIONS::input_vector_t;

  using SYSTEMCOST = CostFunctionBase<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, LOGIC_RULES_T>;

  virtual ~LoopshapingCostImplementationBase() = default;

  virtual void setCurrentStateAndControl(const scalar_t &t, const system_state_vector_t &x_system,
                                         const system_input_vector_t &u_system,
                                         const filter_state_vector_t &x_filter,
                                         const filter_input_vector_t &u_filter) = 0;
  virtual void getIntermediateCost(scalar_t &L) = 0;
  virtual void getIntermediateCostDerivativeTime(scalar_t &dLdt) = 0;
  virtual void getIntermediateCostDerivativeState(state_vector_t &dLdx) = 0;
  virtual void getIntermediateCostSecondDerivativeState(state_matrix_t &dLdxx) = 0;
  virtual void getIntermediateCostDerivativeInput(input_vector_t &dLdu) = 0;
  virtual void getIntermediateCostSecondDerivativeInput(input_matrix_t &dLduu) = 0;
  virtual void getIntermediateCostDerivativeInputState(input_state_matrix_t &dLdux) = 0;
};

}; // namespace ocs2

#endif //OCS2_LOOPSHAPINGCOSTIMPLEMENTATIONBASE_H
