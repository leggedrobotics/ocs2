//
// Created by ruben on 18.09.18.
//

#ifndef OCS2_LOOPSHAPINGCONSTRAINT_H
#define OCS2_LOOPSHAPINGCONSTRAINT_H

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/constraint/ConstraintBase.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"

namespace ocs2 {
template <size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM>
class LoopshapingConstraint : public ConstraintBase<FULL_STATE_DIM, FULL_INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LoopshapingConstraint>;

  using BASE = ConstraintBase<FULL_STATE_DIM, FULL_INPUT_DIM>;
  using typename BASE::constraint1_input_matrix_t;
  using typename BASE::constraint1_state_matrix_t;
  using typename BASE::constraint1_vector_t;
  using typename BASE::constraint2_state_matrix_t;
  using typename BASE::constraint2_vector_t;
  using typename BASE::input_matrix_array_t;
  using typename BASE::input_state_matrix_array_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::state_matrix_array_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;

  using SYSTEM_CONSTRAINT = ConstraintBase<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM>;
  using system_state_vector_t = typename SYSTEM_CONSTRAINT::state_vector_t;
  using system_input_vector_t = typename SYSTEM_CONSTRAINT::input_vector_t;
  using system_contraint1_vector_t = typename SYSTEM_CONSTRAINT::constraint1_vector_t;
  using system_contraint2_vector_t = typename SYSTEM_CONSTRAINT::constraint2_vector_t;
  using system_constraint1_state_matrix_t = typename SYSTEM_CONSTRAINT::constraint1_state_matrix_t;
  using system_constraint1_input_matrix_t = typename SYSTEM_CONSTRAINT::constraint1_input_matrix_t;
  using system_constraint2_state_matrix_t = typename SYSTEM_CONSTRAINT::constraint2_state_matrix_t;
  using system_scalar_array_t = typename SYSTEM_CONSTRAINT::scalar_array_t;
  using system_state_vector_array_t = typename SYSTEM_CONSTRAINT::state_vector_array_t;
  using system_input_vector_array_t = typename SYSTEM_CONSTRAINT::input_vector_array_t;
  using system_state_matrix_array_t = typename SYSTEM_CONSTRAINT::state_matrix_array_t;
  using system_input_matrix_array_t = typename SYSTEM_CONSTRAINT::input_matrix_array_t;
  using system_input_state_matrix_array_t = typename SYSTEM_CONSTRAINT::input_state_matrix_array_t;

  using filter_state_vector_t = Eigen::Matrix<scalar_t, FILTER_STATE_DIM, 1>;
  using filter_input_vector_t = Eigen::Matrix<scalar_t, FILTER_INPUT_DIM, 1>;

  ~LoopshapingConstraint() override = default;

  LoopshapingConstraint(const LoopshapingConstraint& obj)
      : BASE(),
        loopshapingDefinition_(obj.loopshapingDefinition_),
        systemStateInputConstraintApproximationValid_(false),
        systemInequalityConstraintApproximationValid_(false) {
    if (obj.systemConstraint_) {
      systemConstraint_.reset(obj.systemConstraint_->clone());
    }
  }

  static std::unique_ptr<LoopshapingConstraint> create(const SYSTEM_CONSTRAINT& systemConstraint,
                                                       std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  static std::unique_ptr<LoopshapingConstraint> create(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override {
    systemStateInputConstraintApproximationValid_ = false;
    systemInequalityConstraintApproximationValid_ = false;

    BASE::setCurrentStateAndControl(t, x, u);

    t_ = t;
    loopshapingDefinition_->getSystemState(x, x_system_);
    loopshapingDefinition_->getSystemInput(x, u, u_system_);
    loopshapingDefinition_->getFilterState(x, x_filter_);
    loopshapingDefinition_->getFilteredInput(x, u, u_filter_);

    if (systemConstraint_) {
      systemConstraint_->setCurrentStateAndControl(t, x_system_, u_system_);
    }
  }

  void getConstraint1(constraint1_vector_t& e) override {
    size_t numSystemConstraints = 0;
    if (systemConstraint_) {
      numSystemConstraints = systemConstraint_->numStateInputConstraint(BASE::t_);
      system_contraint1_vector_t e_system;
      systemConstraint_->getConstraint1(e_system);
      e.template segment<SYSTEM_INPUT_DIM>(0) = e_system;
    }

    appendConstraint1(numSystemConstraints, e);
  }

  size_t numStateInputConstraint(const scalar_t& time) override {
    size_t numSystemConstraints = 0;
    if (systemConstraint_) {
      numSystemConstraints = systemConstraint_->numStateInputConstraint(time);
    }
    return addNumStateInputConstraint(numSystemConstraints);
  }

  void getConstraint2(constraint2_vector_t& h) override {
    if (systemConstraint_) {
      system_contraint2_vector_t h_system;
      systemConstraint_->getConstraint2(h_system);
      h.template segment<SYSTEM_INPUT_DIM>(0) = h_system;
    }
  }

  size_t numStateOnlyConstraint(const scalar_t& time) override {
    if (systemConstraint_) {
      return systemConstraint_->numStateOnlyConstraint(time);
    } else {
      return 0;
    }
  }

  void getInequalityConstraint(scalar_array_t& h) override {
    if (systemConstraint_) {
      systemConstraint_->getInequalityConstraint(h);
    }
  }

  size_t numInequalityConstraint(const scalar_t& time) override {
    if (systemConstraint_) {
      return systemConstraint_->numInequalityConstraint(time);
    } else {
      return 0;
    }
  }

  void getFinalConstraint2(constraint2_vector_t& h_f) override {
    if (systemConstraint_) {
      system_contraint2_vector_t h_f_system;
      systemConstraint_->getFinalConstraint2(h_f_system);
      h_f.template segment<SYSTEM_INPUT_DIM>(0) = h_f_system;
    }
  }

  size_t numStateOnlyFinalConstraint(const scalar_t& time) override {
    if (systemConstraint_) {
      return systemConstraint_->numStateOnlyFinalConstraint(time);
    } else {
      return 0;
    }
  }

  void getConstraint1DerivativesState(constraint1_state_matrix_t& C) override {
    computeSystemStateInputConstraintDerivatives();

    size_t numSystemConstraints = 0;
    if (systemConstraint_) {
      numSystemConstraints = systemConstraint_->numStateInputConstraint(BASE::t_);
      C.block(0, 0, numSystemConstraints, SYSTEM_STATE_DIM) = C_system.block(0, 0, numSystemConstraints, SYSTEM_STATE_DIM);

      // Add zeros for extra states
      if (FULL_STATE_DIM > SYSTEM_STATE_DIM) {
        C.block(0, SYSTEM_STATE_DIM, numSystemConstraints, FULL_STATE_DIM - SYSTEM_STATE_DIM).setZero();
      }
    }

    appendConstraint1DerivativeState(numSystemConstraints, C);
  }

  void getConstraint1DerivativesControl(constraint1_input_matrix_t& D) override {
    computeSystemStateInputConstraintDerivatives();

    size_t numSystemConstraints = 0;
    if (systemConstraint_) {
      numSystemConstraints = systemConstraint_->numStateInputConstraint(BASE::t_);
      D.block(0, 0, numSystemConstraints, SYSTEM_INPUT_DIM) = D_system.block(0, 0, numSystemConstraints, SYSTEM_INPUT_DIM);

      // Add zeros for extra inputs
      if (FULL_INPUT_DIM > SYSTEM_INPUT_DIM) {
        D.block(0, SYSTEM_INPUT_DIM, numSystemConstraints, FULL_INPUT_DIM - SYSTEM_INPUT_DIM).setZero();
      }
    }

    appendConstraint1DerivativeControl(numSystemConstraints, D);
  }

  void getConstraint2DerivativesState(constraint2_state_matrix_t& F) override {
    if (systemConstraint_) {
      system_constraint2_state_matrix_t F_system;
      systemConstraint_->getConstraint2DerivativesState(F_system);
      F.template block<SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM>(0, 0) = F_system;
      F.template block<SYSTEM_INPUT_DIM, FILTER_STATE_DIM>(0, SYSTEM_STATE_DIM).setZero();
    }
  }

  void getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F_f) {
    if (systemConstraint_) {
      system_constraint2_state_matrix_t F_f_system;
      systemConstraint_->getFinalConstraint2DerivativesState(F_f_system);
      F_f.template block<SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM>(0, 0) = F_f_system;
      F_f.template block<SYSTEM_INPUT_DIM, FILTER_STATE_DIM>(0, SYSTEM_STATE_DIM).setZero();
    }
  }

 protected:
  LoopshapingConstraint(const SYSTEM_CONSTRAINT& systemConstraint, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(),
        systemConstraint_(systemConstraint.clone()),
        loopshapingDefinition_(std::move(loopshapingDefinition)),
        systemStateInputConstraintApproximationValid_(false),
        systemInequalityConstraintApproximationValid_(false){};

  explicit LoopshapingConstraint(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(),
        systemConstraint_(nullptr),
        loopshapingDefinition_(std::move(loopshapingDefinition)),
        systemStateInputConstraintApproximationValid_(false),
        systemInequalityConstraintApproximationValid_(false){};

  std::unique_ptr<SYSTEM_CONSTRAINT> systemConstraint_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;

  scalar_t t_;
  filter_state_vector_t x_filter_;
  filter_input_vector_t u_filter_;
  system_state_vector_t x_system_;
  system_input_vector_t u_system_;

  system_constraint1_state_matrix_t C_system;
  system_constraint1_input_matrix_t D_system;
  void computeSystemStateInputConstraintDerivatives() {
    if (!systemStateInputConstraintApproximationValid_) {
      if (systemConstraint_) {
        systemConstraint_->getConstraint1DerivativesState(C_system);
        systemConstraint_->getConstraint1DerivativesControl(D_system);
      }
      systemStateInputConstraintApproximationValid_ = true;
    }
  }

  system_state_vector_array_t system_dhdx;
  system_input_vector_array_t system_dhdu;
  system_state_matrix_array_t system_ddhdxdx;
  system_input_matrix_array_t system_ddhdudu;
  system_input_state_matrix_array_t system_ddhdudx;
  void computeSystemInequalityConstraintDerivatives() {
    if (!systemInequalityConstraintApproximationValid_) {
      if (systemConstraint_) {
        system_dhdx.clear();
        system_dhdu.clear();
        system_ddhdxdx.clear();
        system_ddhdudu.clear();
        system_ddhdudx.clear();

        systemConstraint_->getInequalityConstraintDerivativesState(system_dhdx);
        systemConstraint_->getInequalityConstraintDerivativesInput(system_dhdu);
        systemConstraint_->getInequalityConstraintSecondDerivativesState(system_ddhdxdx);
        systemConstraint_->getInequalityConstraintSecondDerivativesInput(system_ddhdudu);
        systemConstraint_->getInequalityConstraintDerivativesInputState(system_ddhdudx);
        systemInequalityConstraintApproximationValid_ = true;
      }
    }
  }

 private:
  bool systemStateInputConstraintApproximationValid_;
  bool systemInequalityConstraintApproximationValid_;

  // Interface for derived classes to append additional constraints. Adding nothing by default
  virtual size_t addNumStateInputConstraint(size_t numSystemStateInputConstraints) { return numSystemStateInputConstraints; };
  virtual void appendConstraint1(size_t numSystemStateInputConstraints, constraint1_vector_t& e){};
  virtual void appendConstraint1DerivativeState(size_t numSystemStateInputConstraints, constraint1_state_matrix_t& C){};
  virtual void appendConstraint1DerivativeControl(size_t numSystemStateInputConstraints, constraint1_input_matrix_t& D){};
};
}  // namespace ocs2

#include "ocs2_core/loopshaping/constraint/LoopshapingConstraintEliminatePattern.h"
#include "ocs2_core/loopshaping/constraint/LoopshapingConstraintInputPattern.h"
#include "ocs2_core/loopshaping/constraint/LoopshapingConstraintOutputPattern.h"

// Implement factory method
namespace ocs2 {
template <size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM>
std::unique_ptr<
    LoopshapingConstraint<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>>
LoopshapingConstraint<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>::create(
    std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  switch (loopshapingDefinition->getType()) {
    case LoopshapingType::outputpattern:
      return std::unique_ptr<LoopshapingConstraint>(
          new LoopshapingConstraintOutputPattern<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM,
                                                 FILTER_INPUT_DIM>(std::move(loopshapingDefinition)));
    case LoopshapingType::inputpattern:
      return std::unique_ptr<LoopshapingConstraint>(
          new LoopshapingConstraintInputPattern<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM,
                                                FILTER_INPUT_DIM>(std::move(loopshapingDefinition)));
    case LoopshapingType::eliminatepattern:
      return std::unique_ptr<LoopshapingConstraint>(
          new LoopshapingConstraintEliminatePattern<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM,
                                                    FILTER_INPUT_DIM>(std::move(loopshapingDefinition)));
  }
}

template <size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM>
std::unique_ptr<
    LoopshapingConstraint<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>>
LoopshapingConstraint<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>::create(
    const SYSTEM_CONSTRAINT& systemConstraint, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  switch (loopshapingDefinition->getType()) {
    case LoopshapingType::outputpattern:
      return std::unique_ptr<LoopshapingConstraint>(
          new LoopshapingConstraintOutputPattern<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM,
                                                 FILTER_INPUT_DIM>(systemConstraint, std::move(loopshapingDefinition)));
    case LoopshapingType::inputpattern:
      return std::unique_ptr<LoopshapingConstraint>(
          new LoopshapingConstraintInputPattern<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM,
                                                FILTER_INPUT_DIM>(systemConstraint, std::move(loopshapingDefinition)));
    case LoopshapingType::eliminatepattern:
      return std::unique_ptr<LoopshapingConstraint>(
          new LoopshapingConstraintEliminatePattern<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM,
                                                    FILTER_INPUT_DIM>(systemConstraint, std::move(loopshapingDefinition)));
  }
}
}  // namespace ocs2

#endif  // OCS2_LOOPSHAPINGCONSTRAINT_H
