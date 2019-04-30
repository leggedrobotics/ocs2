//
// Created by rgrandia on 29.04.19.
//

#ifndef OCS2_LOOPSHAPINGCONSTRAINTIMPLEMENTATIONBASE_H
#define OCS2_LOOPSHAPINGCONSTRAINTIMPLEMENTATIONBASE_H

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/constraint/ConstraintBase.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"

namespace ocs2 {
template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
    size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
    size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
    class LOGIC_RULES_T=NullLogicRules>
class LoopshapingConstraintImplementationBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using FULL_DIMENSIONS = ocs2::Dimensions<FULL_STATE_DIM, FULL_INPUT_DIM>;
  using scalar_t = typename FULL_DIMENSIONS::scalar_t;
  using constraint1_vector_t = typename FULL_DIMENSIONS::constraint1_vector_t;
  using constraint1_state_matrix_t = typename FULL_DIMENSIONS::constraint1_state_matrix_t;
  using constraint1_input_matrix_t = typename FULL_DIMENSIONS::constraint1_input_matrix_t;
  using state_vector_array_t = typename FULL_DIMENSIONS::state_vector_array_t;
  using input_vector_array_t = typename FULL_DIMENSIONS::input_vector_array_t;
  using state_matrix_array_t = typename FULL_DIMENSIONS::state_matrix_array_t;
  using input_matrix_array_t = typename FULL_DIMENSIONS::input_matrix_array_t;
  using input_state_matrix_array_t = typename FULL_DIMENSIONS::input_state_matrix_array_t;

  using SYSTEM_DIMENSIONS = ocs2::Dimensions<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM>;
  using system_state_vector_t = typename SYSTEM_DIMENSIONS::state_vector_t;
  using system_input_vector_t = typename SYSTEM_DIMENSIONS::input_vector_t;

  using FILTER_DIMENSIONS = ocs2::Dimensions<FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using filter_state_vector_t = typename FILTER_DIMENSIONS::state_vector_t;
  using filter_input_vector_t = typename FILTER_DIMENSIONS::input_vector_t;

  virtual ~LoopshapingConstraintImplementationBase() = default;

  virtual void setCurrentStateAndControl(const scalar_t &t, const system_state_vector_t &x_system,
                                         const system_input_vector_t &u_system,
                                         const filter_state_vector_t &x_filter,
                                         const filter_input_vector_t &u_filter) = 0;

  virtual size_t numStateInputConstraint(size_t numSystemStateInputConstraints, scalar_t time) = 0;
  virtual void getConstraint1(size_t numSystemStateInputConstraints, constraint1_vector_t &e) = 0;
  virtual void getConstraint1DerivativesState(size_t numSystemStateInputConstraints, constraint1_state_matrix_t &C) = 0;
  virtual void getConstraint1DerivativesControl(size_t numSystemStateInputConstraints, constraint1_input_matrix_t &D) = 0;

  virtual void getInequalityConstraintDerivativesState(state_vector_array_t &dhdx) = 0;
  virtual void getInequalityConstraintDerivativesInput(input_vector_array_t &dhdu) = 0;
  virtual void getInequalityConstraintSecondDerivativesState(state_matrix_array_t &ddhdxdx) = 0;
  virtual void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t &ddhdudu) = 0;
  virtual void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t &ddhdudx) = 0;
};
}; // ocs2


#endif //OCS2_LOOPSHAPINGCONSTRAINTIMPLEMENTATIONBASE_H
