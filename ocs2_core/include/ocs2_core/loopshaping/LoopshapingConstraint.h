//
// Created by ruben on 18.09.18.
//

#ifndef OCS2_LOOPSHAPINGCONSTRAINT_H
#define OCS2_LOOPSHAPINGCONSTRAINT_H

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/constraint/ConstraintBase.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/logic/machine/LogicRulesMachine.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"

namespace ocs2 {
template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
    size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
    size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
    class LOGIC_RULES_T=NullLogicRules>
class LoopshapingConstraint final : public ConstraintBase<
    FULL_STATE_DIM, FULL_INPUT_DIM, LOGIC_RULES_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LoopshapingConstraint>;

  using BASE = ConstraintBase<FULL_STATE_DIM, FULL_INPUT_DIM, LOGIC_RULES_T>;
  using scalar_t = typename BASE::scalar_t;
  using scalar_array_t = typename BASE::scalar_array_t;
  using state_vector_t = typename BASE::state_vector_t;
  using input_vector_t = typename BASE::input_vector_t;
  using state_matrix_t = typename BASE::state_matrix_t;
  using input_matrix_t = typename BASE::input_matrix_t;
  using input_state_matrix_t = typename BASE::input_state_matrix_t;
  using constraint1_vector_t = typename BASE::constraint1_vector_t;
  using constraint2_vector_t = typename BASE::constraint2_vector_t;
  using constraint1_state_matrix_t = typename BASE::constraint1_state_matrix_t;
  using constraint1_input_matrix_t = typename BASE::constraint1_input_matrix_t;
  using constraint2_state_matrix_t = typename BASE::constraint2_state_matrix_t;
  using state_vector_array_t = typename BASE::state_vector_array_t;
  using input_vector_array_t = typename BASE::input_vector_array_t;
  using state_matrix_array_t = typename BASE::state_matrix_array_t;
  using input_matrix_array_t = typename BASE::input_matrix_array_t;
  using input_state_matrix_array_t = typename BASE::input_state_matrix_array_t;

  using SYSTEM_CONSTRAINT = ConstraintBase<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, LOGIC_RULES_T>;
  static constexpr size_t system_state_dim = SYSTEM_STATE_DIM;
  static constexpr size_t system_input_dim = SYSTEM_INPUT_DIM;
  using system_state_vector_t = typename SYSTEM_CONSTRAINT::state_vector_t;
  using system_input_vector_t = typename SYSTEM_CONSTRAINT::input_vector_t;
  using system_state_matrix_t = typename SYSTEM_CONSTRAINT::state_matrix_t;
  using system_input_matrix_t = typename SYSTEM_CONSTRAINT::input_matrix_t;
  using system_state_input_matrix_t = typename SYSTEM_CONSTRAINT::state_input_matrix_t;
  using system_input_state_matrix_t = typename SYSTEM_CONSTRAINT::input_state_matrix_t;
  using system_contraint1_vector_t = typename SYSTEM_CONSTRAINT::constraint1_vector_t;
  using system_contraint2_vector_t = typename SYSTEM_CONSTRAINT::constraint2_vector_t;
  using system_constraint1_state_matrix_t = typename SYSTEM_CONSTRAINT::constraint1_state_matrix_t;
  using system_constraint1_input_matrix_t = typename SYSTEM_CONSTRAINT::constraint1_input_matrix_t;
  using system_constraint2_state_matrix_t = typename SYSTEM_CONSTRAINT::constraint2_state_matrix_t;
  using system_state_vector_array_t = typename SYSTEM_CONSTRAINT::state_vector_array_t;
  using system_input_vector_array_t = typename SYSTEM_CONSTRAINT::input_vector_array_t;
  using system_state_matrix_array_t = typename SYSTEM_CONSTRAINT::state_matrix_array_t;
  using system_input_matrix_array_t = typename SYSTEM_CONSTRAINT::input_matrix_array_t;
  using system_input_state_matrix_array_t = typename SYSTEM_CONSTRAINT::input_state_matrix_array_t;

  static constexpr size_t filter_state_dim = FILTER_STATE_DIM;
  static constexpr size_t filter_input_dim = FILTER_INPUT_DIM;
  size_t filter_constraint_dim = 0;
  using filter_state_vector_t = Eigen::Matrix<scalar_t, filter_state_dim, 1>;
  using filter_input_vector_t = Eigen::Matrix<scalar_t, filter_input_dim, 1>;

  LoopshapingConstraint(const SYSTEM_CONSTRAINT &systemConstraint,
                        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) :
      BASE(),
      systemConstraint_(systemConstraint.clone()),
      loopshapingDefinition_(loopshapingDefinition) {};

  LoopshapingConstraint(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) :
      BASE(),
      systemConstraint_(nullptr),
      loopshapingDefinition_(loopshapingDefinition) {};

  ~LoopshapingConstraint() override = default;

  LoopshapingConstraint(const LoopshapingConstraint &obj) :
      BASE(),
      systemConstraint_(nullptr),
      loopshapingDefinition_(obj.loopshapingDefinition_),
      filter_constraint_dim(obj.filter_constraint_dim) {
    if (obj.systemConstraint_) {
      systemConstraint_ = std::unique_ptr<SYSTEM_CONSTRAINT>(obj.systemConstraint_->clone());
    };
  }

  LoopshapingConstraint *clone() const override {
    return new LoopshapingConstraint(*this);
  };

  virtual void initializeModel(
      LogicRulesMachine <LOGIC_RULES_T> &logicRulesMachine,
      const size_t &partitionIndex,
      const char *algorithmName = NULL) override {
    BASE::initializeModel(logicRulesMachine, partitionIndex, algorithmName);
    if (systemConstraint_) {
      systemConstraint_->initializeModel(logicRulesMachine, partitionIndex, algorithmName);
    };
  }

  virtual void setCurrentStateAndControl(
      const scalar_t &t,
      const state_vector_t &x,
      const input_vector_t &u) override {
    BASE::setCurrentStateAndControl(t, x, u);
    if (systemConstraint_) {
      system_state_vector_t systemstate;
      system_input_vector_t systeminput;
      loopshapingDefinition_->getSystemState(x, systemstate);
      loopshapingDefinition_->getSystemInput(x, u, systeminput);
      systemConstraint_->setCurrentStateAndControl(t, systemstate, systeminput);
    }

    auto &s_filter = loopshapingDefinition_->getInputFilter_s();
    if (s_filter.getNumStates() > 0 && !loopshapingDefinition_->eliminateInputs) {
      filter_constraint_dim = system_input_dim;
    } else {
      filter_constraint_dim = 0;
    }
  }

  virtual void getConstraint1(constraint1_vector_t &e) override {
    const auto &s_filter = loopshapingDefinition_->getInputFilter_s();

    system_state_vector_t systemstate;
    system_input_vector_t systeminput;
    filter_state_vector_t filterstate;
    filter_input_vector_t filteredinput;
    loopshapingDefinition_->getSystemState(BASE::x_, systemstate);
    loopshapingDefinition_->getSystemInput(BASE::x_, BASE::u_, systeminput);
    loopshapingDefinition_->getFilterState(BASE::x_, filterstate);
    loopshapingDefinition_->getFilteredInput(BASE::x_, BASE::u_, filteredinput);

    // in relative coordinates: e = C_s*x_s + D_s*u_s - u
    if (filter_constraint_dim > 0) {
      e.template segment(0, filter_constraint_dim) =
          s_filter.getC() * filterstate
              + s_filter.getD() * filteredinput
              - systeminput;
    }

    if (systemConstraint_) {
      system_contraint1_vector_t e_system;
      systemConstraint_->getConstraint1(e_system);
      e.template segment<system_input_dim>(filter_constraint_dim) = e_system;
    }
  }

  virtual size_t numStateInputConstraint(const scalar_t &time) override {
    if (systemConstraint_) {
      return filter_constraint_dim + systemConstraint_->numStateInputConstraint(time);
    } else {
      return filter_constraint_dim;
    }
  }

  virtual void getConstraint2(constraint2_vector_t &h) override {
    if (systemConstraint_) {
      system_contraint2_vector_t h_system;
      systemConstraint_->getConstraint2(h_system);
      h.template segment<system_input_dim>(0) = h_system;
    }
  }

  virtual size_t numStateOnlyConstraint(const scalar_t &time) override {
    if (systemConstraint_) {
      return systemConstraint_->numStateOnlyConstraint(time);
    } else {
      return 0;
    }
  }

  virtual void getInequalityConstraint(scalar_array_t &h) override {
    if (systemConstraint_) {
      systemConstraint_->getInequalityConstraint(h);
    }
  }

  virtual size_t numInequalityConstraint(const scalar_t &time) override {
    if (systemConstraint_) {
      return systemConstraint_->numInequalityConstraint(time);
    } else {
      return 0;
    }
  }

  virtual void getFinalConstraint2(constraint2_vector_t &h_f) override {
    if (systemConstraint_) {
      system_contraint2_vector_t h_f_system;
      systemConstraint_->getFinalConstraint2(h_f_system);
      h_f.template segment<system_input_dim>(0) = h_f_system;
    }
  }

  virtual size_t numStateOnlyFinalConstraint(const scalar_t &time) override {
    if (systemConstraint_) {
      return systemConstraint_->numStateOnlyFinalConstraint(time);
    } else {
      return 0;
    }
  }

  virtual void getConstraint1DerivativesState(constraint1_state_matrix_t &C) override {
    /*
     *  C = [0         C_filter;
     *       C_system  0  ]
     *    = [filter_out_dim x system_state_dim,    filter_out_dim x filterstate_dim
     *       system_input_dim x system_state_dim,  system_input_dim x filterstate_dim
     */
    const auto &s_filter = loopshapingDefinition_->getInputFilter_s();

    if (filter_constraint_dim > 0) {
      C.template block(0, 0, filter_constraint_dim, system_state_dim).setZero();
      C.template block(0, system_state_dim, filter_constraint_dim, s_filter.getNumStates()) = s_filter.getC();
    }

    if (systemConstraint_) {
      auto numSystemConstraints = systemConstraint_->numStateInputConstraint(BASE::t_);
      system_constraint1_state_matrix_t C_system;
      systemConstraint_->getConstraint1DerivativesState(C_system);
      C.template block(filter_constraint_dim, 0, numSystemConstraints, system_state_dim) =
          C_system.block(0, 0, numSystemConstraints, system_state_dim);
      if (loopshapingDefinition_->eliminateInputs) {
        system_constraint1_input_matrix_t D_system;
        systemConstraint_->getConstraint1DerivativesControl(D_system);
        C.template block(filter_constraint_dim, system_state_dim, numSystemConstraints, filter_state_dim) =
            D_system.block(0, 0, numSystemConstraints, system_input_dim) * s_filter.getC();
      } else {
        C.template block(filter_constraint_dim, system_state_dim, numSystemConstraints, filter_state_dim).setZero();
      }
    }
  }

  virtual void getConstraint1DerivativesControl(constraint1_input_matrix_t &D) override {
    /*
     *  D = [-I      D_filter
     *       D_system  0]
     *    = [filter_out_dim x system_input_dim, filter_out_dim x filter_input_dim
     *       system_input_dim x system_input_dim, system_input_dim x filter_input_dim
     */
    const auto &s_filter = loopshapingDefinition_->getInputFilter_s();

    if (filter_constraint_dim) {
      D.template block(0, 0, filter_constraint_dim, system_input_dim) =
          -Eigen::Matrix<scalar_t, -1, -1>::Identity(filter_constraint_dim, system_input_dim);
      D.template block(0, system_input_dim, filter_constraint_dim, filter_input_dim) = s_filter.getD();
    }

    if (systemConstraint_) {
      auto numSystemConstraints = systemConstraint_->numStateInputConstraint(BASE::t_);
      system_constraint1_input_matrix_t D_system;
      systemConstraint_->getConstraint1DerivativesControl(D_system);
      if (loopshapingDefinition_->eliminateInputs) {
        D.template block(filter_constraint_dim, 0, numSystemConstraints, system_input_dim) =
            D_system.block(0, 0, numSystemConstraints, system_input_dim) * s_filter.getD();
      } else {
        D.template block(filter_constraint_dim, 0, numSystemConstraints, system_input_dim) =
            D_system.block(0, 0, numSystemConstraints, system_input_dim);
        D.template block(filter_constraint_dim, system_input_dim, numSystemConstraints, filter_input_dim).setZero();
      }
    }
  }

  virtual void getConstraint2DerivativesState(constraint2_state_matrix_t &F) override {
    if (systemConstraint_) {
      system_constraint2_state_matrix_t F_system;
      systemConstraint_->getConstraint2DerivativesState(F_system);
      F.template block<system_input_dim, system_state_dim>(0, 0) = F_system;
      F.template block<system_input_dim, filter_state_dim>(0, system_state_dim).setZero();
    }
  }

  virtual void getInequalityConstraintDerivativesState(state_vector_array_t &dhdx) override {
    dhdx.clear();
    if (systemConstraint_) {
      const auto nIneq = numInequalityConstraint(BASE::t_);
      dhdx.resize(nIneq);

      // Compute system inequality derivatives
      system_state_vector_array_t system_dhdx;
      systemConstraint_->getInequalityConstraintDerivativesState(system_dhdx);

      const auto &s_filter = loopshapingDefinition_->getInputFilter_s();

      if (s_filter.getNumStates() > 0 && loopshapingDefinition_->eliminateInputs){
        system_input_vector_array_t system_dhdu;
        systemConstraint_->getInequalityConstraintDerivativesInput(system_dhdu);
        for (size_t i = 0; i < nIneq; i++) {
          loopshapingDefinition_->functionDerivativeState(system_dhdx[i], system_dhdu[i], dhdx[i]);
        }
      } else {
        for (size_t i = 0; i < nIneq; i++) {
          loopshapingDefinition_->functionDerivativeState(system_dhdx[i], dhdx[i]);
        }
      }
    }
  }

  virtual void getInequalityConstraintDerivativesInput(input_vector_array_t &dhdu) override {
    dhdu.clear();
    if (systemConstraint_) {
      const auto nIneq = numInequalityConstraint(BASE::t_);
      dhdu.resize(nIneq);

      // Compute system inequality derivatives
      system_input_vector_array_t system_dhdu;
      systemConstraint_->getInequalityConstraintDerivativesInput(system_dhdu);

      for (size_t i = 0; i < nIneq; i++) {
        loopshapingDefinition_->functionDerivativeInput(system_dhdu[i], dhdu[i]);
      }
    }
  }

  virtual void getInequalityConstraintSecondDerivativesState(state_matrix_array_t &ddhdxdx) override {
    ddhdxdx.clear();
    if (systemConstraint_) {
      const auto nIneq = numInequalityConstraint(BASE::t_);
      ddhdxdx.resize(nIneq);

      // Compute system inequality constraint hessians
      system_state_matrix_array_t system_ddhdxdx;
      systemConstraint_->getInequalityConstraintSecondDerivativesState(system_ddhdxdx);

      const auto &s_filter = loopshapingDefinition_->getInputFilter_s();

      if (s_filter.getNumStates() > 0 && loopshapingDefinition_->eliminateInputs){
        system_input_matrix_array_t system_ddhdudu;
        system_input_state_matrix_array_t system_ddhdudx;
        systemConstraint_->getInequalityConstraintSecondDerivativesInput(system_ddhdudu);
        systemConstraint_->getInequalityConstraintDerivativesInputState(system_ddhdudx);
        for (size_t i = 0; i < nIneq; i++) {
          loopshapingDefinition_->functionSecondDerivativeState(system_ddhdxdx[i], system_ddhdudx[i], system_ddhdudu[i], ddhdxdx[i]);
        }
      } else {
        for (size_t i = 0; i < nIneq; i++) {
          loopshapingDefinition_->functionSecondDerivativeState(system_ddhdxdx[i], ddhdxdx[i]);
        }
      }
    }
  }

  virtual void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t &ddhdudu) {
    ddhdudu.clear();
    if (systemConstraint_) {
      const auto nIneq = numInequalityConstraint(BASE::t_);
      ddhdudu.resize(nIneq);

      // Compute system constraint hessians
      system_input_matrix_array_t system_ddhdudu;
      systemConstraint_->getInequalityConstraintSecondDerivativesInput(system_ddhdudu);

      for (size_t i = 0; i < nIneq; i++) {
        loopshapingDefinition_->functionSecondDerivativeInput(system_ddhdudu[i], ddhdudu[i]);
      }
    }
  }

  virtual void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t &ddhdudx) {
    ddhdudx.clear();
    if (systemConstraint_) {
      const auto nIneq = numInequalityConstraint(BASE::t_);
      ddhdudx.resize(nIneq);

      // Compute system hessians
      system_input_state_matrix_array_t system_ddhdudx;
      systemConstraint_->getInequalityConstraintDerivativesInputState(system_ddhdudx);

      const auto &s_filter = loopshapingDefinition_->getInputFilter_s();

      if (s_filter.getNumStates() > 0 && loopshapingDefinition_->eliminateInputs){
        system_input_matrix_array_t system_ddhdudu;
        systemConstraint_->getInequalityConstraintSecondDerivativesInput(system_ddhdudu);
        for (size_t i = 0; i < nIneq; i++) {
          loopshapingDefinition_->functionDerivativeInputState(system_ddhdudx[i], system_ddhdudu[i], ddhdudx[i]);
        }
      } else {
        for (size_t i = 0; i < nIneq; i++) {
          loopshapingDefinition_->functionDerivativeInputState(system_ddhdudx[i], ddhdudx[i]);
        }
      }
    }
  }

  virtual void getFinalConstraint2DerivativesState(constraint2_state_matrix_t &F_f) {
    if (systemConstraint_) {
      system_constraint2_state_matrix_t F_f_system;
      systemConstraint_->getFinalConstraint2DerivativesState(F_f_system);
      F_f.template block<system_input_dim, system_state_dim>(0, 0) = F_f_system;
      F_f.template block<system_input_dim, filter_state_dim>(0, system_state_dim).setZero();
    }
  }

 private:
  std::unique_ptr<SYSTEM_CONSTRAINT> systemConstraint_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};
} // namespace ocs2;

#endif //OCS2_LOOPSHAPINGCONSTRAINT_H
