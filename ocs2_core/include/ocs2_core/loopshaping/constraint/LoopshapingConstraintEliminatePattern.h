//
// Created by rgrandia on 29.04.19.
//

#ifndef OCS2_LOOPSHAPINGCONSTRAINTELIMINATEPATTERN_H
#define OCS2_LOOPSHAPINGCONSTRAINTELIMINATEPATTERN_H

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/constraint/ConstraintBase.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"
#include "ocs2_core/loopshaping/constraint/LoopshapingConstraintImplementationBase.h"

namespace ocs2 {
template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
    size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
    size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
    class LOGIC_RULES_T=NullLogicRules>
class LoopshapingConstraintEliminatePattern final: public LoopshapingConstraintImplementationBase<FULL_STATE_DIM, FULL_INPUT_DIM,
                                                                                              SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM,
                                                                                              FILTER_STATE_DIM, FILTER_INPUT_DIM, NullLogicRules> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

  LoopshapingConstraintEliminatePattern( std::shared_ptr<SYSTEM_CONSTRAINT> systemConstraint,
                                     std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) :
      systemConstraint_(systemConstraint), loopshapingDefinition_(loopshapingDefinition) {};

  virtual ~LoopshapingConstraintEliminatePattern() = default;

  virtual void setCurrentStateAndControl(const scalar_t &t, const system_state_vector_t &x_system,
                                         const system_input_vector_t &u_system,
                                         const filter_state_vector_t &x_filter,
                                         const filter_input_vector_t &u_filter) override {
    t_ = t;
    x_system_ = x_system;
    u_system_ = u_system;
    x_filter_ = x_filter;
    u_filter_ = u_filter;
  };

  virtual size_t numStateInputConstraint(size_t numSystemStateInputConstraints, scalar_t time) override {
    return numSystemStateInputConstraints + SYSTEM_INPUT_DIM;
  };

  virtual void getConstraint1(size_t numSystemStateInputConstraints, constraint1_vector_t &e) override {

  };

  virtual void getConstraint1DerivativesState(size_t numSystemStateInputConstraints, constraint1_state_matrix_t &C) override {
    const auto &s_filter = loopshapingDefinition_->getInputFilter_s();
    C.block(numSystemStateInputConstraints, 0, SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM).setZero();
    C.block(numSystemStateInputConstraints, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM) = s_filter.getC();
    // TODO
  };

  virtual void getConstraint1DerivativesControl(size_t numSystemStateInputConstraints, constraint1_input_matrix_t &D) override {
    const auto &s_filter = loopshapingDefinition_->getInputFilter_s();
    D.block(numSystemStateInputConstraints, 0, SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM) =
        -Eigen::Matrix<scalar_t, SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM>::Identity();
    D.block(numSystemStateInputConstraints, SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM, FILTER_INPUT_DIM) = s_filter.getD();
    // TODO
  };

  virtual void getInequalityConstraintDerivativesState(state_vector_array_t &dhdx) override {
    const auto &s_filter = loopshapingDefinition_->getInputFilter_s();

    dhdx.clear();
    if (systemConstraint_) {
      // Compute system inequality derivatives
      system_state_vector_array_t system_dhdx;
      systemConstraint_->getInequalityConstraintDerivativesState(system_dhdx);
      system_input_vector_array_t system_dhdu;
      systemConstraint_->getInequalityConstraintDerivativesInput(system_dhdu);

      const auto nIneq = systemConstraint_->numInequalityConstraint(t_);
      dhdx.resize(nIneq);

      for (size_t i = 0; i < nIneq; i++) {
        dhdx[i].head(system_dhdx[i].size()) = system_dhdx[i];
        dhdx[i].tail(FILTER_STATE_DIM) = s_filter.getC().transpose() * system_dhdu[i];
      }
    }
  };

  virtual void getInequalityConstraintDerivativesInput(input_vector_array_t &dhdu) override {
    const auto &s_filter = loopshapingDefinition_->getInputFilter_s();

    dhdu.clear();
    if (systemConstraint_) {
      // Compute system inequality derivatives
      system_input_vector_array_t system_dhdu;
      systemConstraint_->getInequalityConstraintDerivativesInput(system_dhdu);

      const auto nIneq = systemConstraint_->numInequalityConstraint(t_);
      dhdu.resize(nIneq);

      for (size_t i = 0; i < nIneq; i++) {
        dhdu[i].head(FILTER_INPUT_DIM) = s_filter.getD().transpose() * system_dhdu[i];
      }
    }
  };

  virtual void getInequalityConstraintSecondDerivativesState(state_matrix_array_t &ddhdxdx) override {
    const auto &s_filter = loopshapingDefinition_->getInputFilter_s();

    ddhdxdx.clear();
    if (systemConstraint_) {
      // Compute system inequality constraint hessians
      system_state_matrix_array_t system_ddhdxdx;
      system_input_matrix_array_t system_ddhdudu;
      system_input_state_matrix_array_t system_ddhdudx;
      systemConstraint_->getInequalityConstraintSecondDerivativesState(system_ddhdxdx);
      systemConstraint_->getInequalityConstraintSecondDerivativesInput(system_ddhdudu);
      systemConstraint_->getInequalityConstraintDerivativesInputState(system_ddhdudx);

      const auto nIneq = systemConstraint_->numInequalityConstraint(t_);
      ddhdxdx.resize(nIneq);
      for (size_t i = 0; i < nIneq; i++) {
        ddhdxdx[i].block(0, 0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = system_ddhdxdx[i];
        ddhdxdx[i].block(0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM) = system_ddhdudx[i].transpose() * s_filter.getC();;
        ddhdxdx[i].block(SYSTEM_STATE_DIM, 0, FILTER_STATE_DIM, SYSTEM_STATE_DIM) = ddhdxdx[i].block(0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM).transpose();
        ddhdxdx[i].block(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM, FILTER_STATE_DIM) = s_filter.getC().transpose() * system_ddhdudu[i] * s_filter.getC();
      }
    }
  };

  virtual void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t &ddhdudu) override {
    const auto &s_filter = loopshapingDefinition_->getInputFilter_s();

    ddhdudu.clear();
    if (systemConstraint_) {
      // Compute system constraint hessians
      system_input_matrix_array_t system_ddhdudu;
      systemConstraint_->getInequalityConstraintSecondDerivativesInput(system_ddhdudu);

      const auto nIneq = systemConstraint_->numInequalityConstraint(t_);
      ddhdudu.resize(nIneq);

      for (size_t i = 0; i < nIneq; i++) {
        ddhdudu[i].block(0, 0, SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM) = s_filter.getD().transpose() * system_ddhdudu[i] * s_filter.getD();
      }
    }
  };

  virtual void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t &ddhdudx) override {
    const auto &s_filter = loopshapingDefinition_->getInputFilter_s();

    ddhdudx.clear();
    if (systemConstraint_) {
      // Compute system hessians
      system_input_state_matrix_array_t system_ddhdudx;
      system_input_matrix_array_t system_ddhdudu;
      systemConstraint_->getInequalityConstraintDerivativesInputState(system_ddhdudx);
      systemConstraint_->getInequalityConstraintSecondDerivativesInput(system_ddhdudu);

      const auto nIneq = systemConstraint_->numInequalityConstraint(t_);
      ddhdudx.resize(nIneq);

      for (size_t i = 0; i < nIneq; i++) {
        ddhdudx[i].block(0, 0, FILTER_INPUT_DIM, SYSTEM_STATE_DIM) = s_filter.getD().transpose() * system_ddhdudx[i];
        ddhdudx[i].block(0, SYSTEM_STATE_DIM, FILTER_INPUT_DIM, FILTER_STATE_DIM) =  s_filter.getD().transpose() * system_ddhdudx[i] * s_filter.getC();
        // TODO: wrong? ddhdudu part missing?
      }
    }
  };

 private:
  std::shared_ptr<SYSTEM_CONSTRAINT> systemConstraint_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;

  scalar_t  t_;
  filter_state_vector_t x_filter_;
  filter_input_vector_t u_filter_;
  system_state_vector_t x_system_;
  system_input_vector_t u_system_;
};
}; // ocs2

#endif //OCS2_LOOPSHAPINGCONSTRAINTELIMINATEPATTERN_H
