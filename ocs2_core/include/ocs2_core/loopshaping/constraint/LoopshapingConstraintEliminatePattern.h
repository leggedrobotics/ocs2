

#ifndef OCS2_LOOPSHAPINGCONSTRAINTELIMINATEPATTERN_H
#define OCS2_LOOPSHAPINGCONSTRAINTELIMINATEPATTERN_H

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/constraint/ConstraintBase.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"

namespace ocs2 {
template <size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM>
class LoopshapingConstraintEliminatePattern final
    : public LoopshapingConstraint<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE =
      LoopshapingConstraint<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;

  using typename BASE::constraint1_input_matrix_t;
  using typename BASE::constraint1_state_matrix_t;
  using typename BASE::constraint1_vector_t;
  using typename BASE::constraint2_vector_t;
  using typename BASE::input_matrix_array_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_array_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::state_matrix_array_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;

  using typename BASE::SYSTEM_CONSTRAINT;
  using typename BASE::system_constraint1_input_matrix_t;
  using typename BASE::system_constraint1_state_matrix_t;
  using typename BASE::system_constraint2_state_matrix_t;
  using typename BASE::system_contraint1_vector_t;
  using typename BASE::system_contraint2_vector_t;
  using typename BASE::system_input_matrix_array_t;
  using typename BASE::system_input_state_matrix_array_t;
  using typename BASE::system_input_vector_array_t;
  using typename BASE::system_input_vector_t;
  using typename BASE::system_scalar_array_t;
  using typename BASE::system_state_matrix_array_t;
  using typename BASE::system_state_vector_array_t;
  using typename BASE::system_state_vector_t;

  using typename BASE::filter_input_vector_t;
  using typename BASE::filter_state_vector_t;

  explicit LoopshapingConstraintEliminatePattern(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(std::move(loopshapingDefinition)){};

  LoopshapingConstraintEliminatePattern(const SYSTEM_CONSTRAINT& systemConstraint,
                                        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemConstraint, std::move(loopshapingDefinition)){};

  virtual ~LoopshapingConstraintEliminatePattern() = default;

  LoopshapingConstraintEliminatePattern(const LoopshapingConstraintEliminatePattern& obj) = default;

  LoopshapingConstraintEliminatePattern* clone() const override { return new LoopshapingConstraintEliminatePattern(*this); };

  void getInequalityConstraintDerivativesState(state_vector_array_t& dhdx) override {
    this->computeSystemInequalityConstraintDerivatives();
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    dhdx.clear();
    if (systemConstraint_) {
      dhdx.resize(system_dhdx.size());
      for (size_t i = 0; i < system_dhdx.size(); i++) {
        dhdx[i].head(SYSTEM_STATE_DIM) = system_dhdx[i];
        dhdx[i].tail(FILTER_STATE_DIM).noalias() = s_filter.getC().transpose() * system_dhdu[i];
      }
    }
  };

  void getInequalityConstraintDerivativesInput(input_vector_array_t& dhdu) override {
    this->computeSystemInequalityConstraintDerivatives();
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    dhdu.clear();
    if (systemConstraint_) {
      dhdu.resize(system_dhdu.size());
      for (size_t i = 0; i < system_dhdu.size(); i++) {
        dhdu[i].noalias() = s_filter.getD().transpose() * system_dhdu[i];
      }
    }
  };

  void getInequalityConstraintSecondDerivativesState(state_matrix_array_t& ddhdxdx) override {
    this->computeSystemInequalityConstraintDerivatives();
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    ddhdxdx.clear();
    if (systemConstraint_) {
      ddhdxdx.resize(system_ddhdxdx.size());
      for (size_t i = 0; i < system_ddhdxdx.size(); i++) {
        ddhdxdx[i].block(0, 0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = system_ddhdxdx[i];
        ddhdxdx[i].block(0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM).noalias() =
            system_ddhdudx[i].transpose() * s_filter.getC();
        ddhdxdx[i].block(SYSTEM_STATE_DIM, 0, FILTER_STATE_DIM, SYSTEM_STATE_DIM) =
            ddhdxdx[i].block(0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM).transpose();
        ddhdxdx[i].block(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM, FILTER_STATE_DIM).noalias() =
            s_filter.getC().transpose() * system_ddhdudu[i] * s_filter.getC();
      }
    }
  };

  void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t& ddhdudu) override {
    this->computeSystemInequalityConstraintDerivatives();
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    ddhdudu.clear();
    if (systemConstraint_) {
      ddhdudu.resize(system_ddhdudu.size());
      for (size_t i = 0; i < system_ddhdudu.size(); i++) {
        ddhdudu[i].noalias() = s_filter.getD().transpose() * system_ddhdudu[i] * s_filter.getD();
      }
    }
  };

  void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t& ddhdudx) override {
    this->computeSystemInequalityConstraintDerivatives();
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    ddhdudx.clear();
    if (systemConstraint_) {
      ddhdudx.resize(system_ddhdudx.size());
      for (size_t i = 0; i < system_ddhdudx.size(); i++) {
        ddhdudx[i].block(0, 0, SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM).noalias() = s_filter.getD().transpose() * system_ddhdudx[i];
        ddhdudx[i].block(0, SYSTEM_STATE_DIM, FILTER_INPUT_DIM, FILTER_STATE_DIM).noalias() =
            s_filter.getD().transpose() * system_ddhdudu[i] * s_filter.getC();
      }
    }
  };

 protected:
  using BASE::loopshapingDefinition_;
  using BASE::systemConstraint_;

  using BASE::t_;
  using BASE::u_filter_;
  using BASE::u_system_;
  using BASE::x_filter_;
  using BASE::x_system_;

  using BASE::C_system;
  using BASE::D_system;

  using BASE::system_ddhdudu;
  using BASE::system_ddhdudx;
  using BASE::system_ddhdxdx;
  using BASE::system_dhdu;
  using BASE::system_dhdx;

 private:
  void appendConstraint1DerivativeState(size_t numSystemStateInputConstraints, constraint1_state_matrix_t& C) override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    // C.block(0, 0, numSystemStateInputConstraints, SYSTEM_STATE_DIM), stays unaltered
    C.block(0, SYSTEM_STATE_DIM, numSystemStateInputConstraints, FILTER_STATE_DIM).noalias() =
        D_system.block(0, 0, numSystemStateInputConstraints, SYSTEM_INPUT_DIM) * s_filter.getC();
  };

  void appendConstraint1DerivativeControl(size_t numSystemStateInputConstraints, constraint1_input_matrix_t& D) override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    D.block(0, 0, numSystemStateInputConstraints, FILTER_INPUT_DIM).noalias() =
        D_system.block(0, 0, numSystemStateInputConstraints, SYSTEM_INPUT_DIM) * s_filter.getD();
  };
};
}  // namespace ocs2

#endif  // OCS2_LOOPSHAPINGCONSTRAINTELIMINATEPATTERN_H
