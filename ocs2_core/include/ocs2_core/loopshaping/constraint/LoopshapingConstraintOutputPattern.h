//
// Created by rgrandia on 29.04.19.
//

#ifndef OCS2_LOOPSHAPINGCONSTRAINTOUTPUTPATTERN_H
#define OCS2_LOOPSHAPINGCONSTRAINTOUTPUTPATTERN_H

namespace ocs2 {
template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
    size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
    size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
    class LOGIC_RULES_T=NullLogicRules>
class LoopshapingConstraintOutputPattern final: public LoopshapingConstraint<FULL_STATE_DIM, FULL_INPUT_DIM,
                                                                                              SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM,
                                                                                              FILTER_STATE_DIM, FILTER_INPUT_DIM, LOGIC_RULES_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = LoopshapingConstraint<FULL_STATE_DIM, FULL_INPUT_DIM,
                                     SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM,
                                     FILTER_STATE_DIM, FILTER_INPUT_DIM, LOGIC_RULES_T>;

  using FULL_DIMENSIONS = ocs2::Dimensions<FULL_STATE_DIM, FULL_INPUT_DIM>;
  using scalar_t = typename FULL_DIMENSIONS::scalar_t;
  using state_vector_t = typename FULL_DIMENSIONS::state_vector_t;
  using input_vector_t = typename FULL_DIMENSIONS::input_vector_t;
  using state_matrix_t = typename FULL_DIMENSIONS::state_matrix_t;
  using input_matrix_t = typename FULL_DIMENSIONS::input_matrix_t;
  using input_state_matrix_t = typename FULL_DIMENSIONS::input_state_matrix_t;
  using constraint1_vector_t = typename FULL_DIMENSIONS::constraint1_vector_t;
  using constraint2_vector_t = typename FULL_DIMENSIONS::constraint2_vector_t;
  using constraint1_state_matrix_t = typename FULL_DIMENSIONS::constraint1_state_matrix_t;
  using constraint1_input_matrix_t = typename FULL_DIMENSIONS::constraint1_input_matrix_t;
  using scalar_array_t = typename FULL_DIMENSIONS::scalar_array_t;
  using state_vector_array_t = typename FULL_DIMENSIONS::state_vector_array_t;
  using input_vector_array_t = typename FULL_DIMENSIONS::input_vector_array_t;
  using state_matrix_array_t = typename FULL_DIMENSIONS::state_matrix_array_t;
  using input_matrix_array_t = typename FULL_DIMENSIONS::input_matrix_array_t;
  using input_state_matrix_array_t = typename FULL_DIMENSIONS::input_state_matrix_array_t;

  using SYSTEM_DIMENSIONS = ocs2::Dimensions<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM>;
  using system_state_vector_t = typename SYSTEM_DIMENSIONS::state_vector_t;
  using system_input_vector_t = typename SYSTEM_DIMENSIONS::input_vector_t;
  using system_state_matrix_t = typename SYSTEM_DIMENSIONS::state_matrix_t;
  using system_input_matrix_t = typename SYSTEM_DIMENSIONS::input_matrix_t;
  using system_input_state_matrix_t = typename SYSTEM_DIMENSIONS::input_state_matrix_t;
  using system_state_vector_array_t = typename SYSTEM_DIMENSIONS::state_vector_array_t;
  using system_input_vector_array_t = typename SYSTEM_DIMENSIONS::input_vector_array_t;
  using system_state_matrix_array_t = typename SYSTEM_DIMENSIONS::state_matrix_array_t;
  using system_input_matrix_array_t = typename SYSTEM_DIMENSIONS::input_matrix_array_t;
  using system_input_state_matrix_array_t = typename SYSTEM_DIMENSIONS::input_state_matrix_array_t;

  using FILTER_DIMENSIONS = ocs2::Dimensions<FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using filter_state_vector_t = typename FILTER_DIMENSIONS::state_vector_t;
  using filter_input_vector_t = typename FILTER_DIMENSIONS::input_vector_t;

  using SYSTEM_CONSTRAINT = ConstraintBase<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, LOGIC_RULES_T>;

  LoopshapingConstraintOutputPattern( std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) : BASE(std::move(loopshapingDefinition)) {};

  LoopshapingConstraintOutputPattern(const SYSTEM_CONSTRAINT &systemConstraint,
                                     std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) : BASE(systemConstraint, std::move(loopshapingDefinition)) {};

  virtual ~LoopshapingConstraintOutputPattern() = default;

  LoopshapingConstraintOutputPattern(const LoopshapingConstraintOutputPattern &obj) = default;

  LoopshapingConstraintOutputPattern* clone() const override {
    return new LoopshapingConstraintOutputPattern(*this);
  };

  void getInequalityConstraintDerivativesState(state_vector_array_t &dhdx) override {
    dhdx.clear();
    if (systemConstraint_) {
      // Compute system inequality derivatives
      system_state_vector_array_t system_dhdx;
      systemConstraint_->getInequalityConstraintDerivativesState(system_dhdx);

      dhdx.resize(system_dhdx.size());
      for (size_t i = 0; i < system_dhdx.size(); i++) {
        dhdx[i].head(system_dhdx[i].size()) = system_dhdx[i];
        dhdx[i].tail(FILTER_STATE_DIM).setZero();
      }
    }
  };

  void getInequalityConstraintDerivativesInput(input_vector_array_t &dhdu) override {
    dhdu.clear();
    if (systemConstraint_) {
      // Compute system inequality derivatives
      system_input_vector_array_t system_dhdu;
      systemConstraint_->getInequalityConstraintDerivativesInput(system_dhdu);

      dhdu.resize(system_dhdu.size());
      for (size_t i = 0; i < system_dhdu.size(); i++) {
        dhdu[i].head(system_dhdu[i].size()) = system_dhdu[i];
      }
    }
  };

  void getInequalityConstraintSecondDerivativesState(state_matrix_array_t &ddhdxdx) override {
    ddhdxdx.clear();
    if (systemConstraint_) {
      // Compute system inequality constraint hessians
      system_state_matrix_array_t system_ddhdxdx;
      systemConstraint_->getInequalityConstraintSecondDerivativesState(system_ddhdxdx);

      ddhdxdx.resize(system_ddhdxdx.size());
      for (size_t i = 0; i < system_ddhdxdx.size(); i++) {
        ddhdxdx[i].block(0, 0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = system_ddhdxdx[i];
        ddhdxdx[i].block(0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM).setZero();
        ddhdxdx[i].block(SYSTEM_STATE_DIM, 0, FILTER_STATE_DIM, SYSTEM_STATE_DIM).setZero();
        ddhdxdx[i].block(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM, FILTER_STATE_DIM).setZero();
      }
    }
  };

  void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t &ddhdudu) override {
    ddhdudu.clear();
    if (systemConstraint_) {
      // Compute system constraint hessians
      system_input_matrix_array_t system_ddhdudu;
      systemConstraint_->getInequalityConstraintSecondDerivativesInput(system_ddhdudu);

      ddhdudu.resize(system_ddhdudu.size());
      for (size_t i = 0; i < system_ddhdudu.size(); i++) {
        ddhdudu[i].block(0, 0, SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM) = system_ddhdudu[i];
      }
    }
  };

  void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t &ddhdudx) override {
    ddhdudx.clear();
    if (systemConstraint_) {
      // Compute system hessians
      system_input_state_matrix_array_t system_ddhdudx;
      systemConstraint_->getInequalityConstraintDerivativesInputState(system_ddhdudx);

      ddhdudx.resize(system_ddhdudx.size());
      for (size_t i = 0; i < system_ddhdudx.size(); i++) {
        ddhdudx[i].block(0, 0, SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM) = system_ddhdudx[i];
        ddhdudx[i].block(0, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM).setZero();
      }
    }
  };

 protected:
  using BASE::systemConstraint_;
  using BASE::loopshapingDefinition_;

  using BASE::t_;
  using BASE::x_filter_;
  using BASE::u_filter_;
  using BASE::x_system_;
  using BASE::u_system_;
};
}; // ocs2

#endif //OCS2_LOOPSHAPINGCONSTRAINTOUTPUTPATTERN_H
