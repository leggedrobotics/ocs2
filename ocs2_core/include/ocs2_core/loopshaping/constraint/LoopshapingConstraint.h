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

#include "ocs2_core/loopshaping/constraint/LoopshapingConstraintImplementationBase.h"
#include "ocs2_core/loopshaping/constraint/LoopshapingConstraintEliminatePattern.h"
#include "ocs2_core/loopshaping/constraint/LoopshapingConstraintInputPattern.h"
#include "ocs2_core/loopshaping/constraint/LoopshapingConstraintOutputPattern.h"

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

  using LoopshapingConstraintImplementation = LoopshapingConstraintImplementationBase<FULL_STATE_DIM, FULL_INPUT_DIM,
                                                                          SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM,
                                                                          FILTER_STATE_DIM, FILTER_INPUT_DIM, NullLogicRules>;

  LoopshapingConstraint(const SYSTEM_CONSTRAINT &systemConstraint,
                        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) :
      BASE(),
      systemConstraint_(systemConstraint.clone()),
      loopshapingDefinition_(loopshapingDefinition) {
    // TODO(Ruben): initialize safely elsewhere
    if (loopshapingDefinition_->getInputFilter_s().getNumOutputs() > 0) {
      if (loopshapingDefinition_->eliminateInputs){
        loopshapingConstraintImplementation_.reset(new LoopshapingConstraintEliminatePattern<FULL_STATE_DIM,
                                                                                         FULL_INPUT_DIM,
                                                                                         SYSTEM_STATE_DIM,
                                                                                         SYSTEM_INPUT_DIM,
                                                                                         FILTER_STATE_DIM,
                                                                                         FILTER_INPUT_DIM,
                                                                                         NullLogicRules>(systemConstraint_,
                                                                                                         loopshapingDefinition_));
      } else {
        loopshapingConstraintImplementation_.reset(new LoopshapingConstraintInputPattern<FULL_STATE_DIM,
                                                                                         FULL_INPUT_DIM,
                                                                                         SYSTEM_STATE_DIM,
                                                                                         SYSTEM_INPUT_DIM,
                                                                                         FILTER_STATE_DIM,
                                                                                         FILTER_INPUT_DIM,
                                                                                         NullLogicRules>(systemConstraint_,
                                                                                                         loopshapingDefinition_));
      }
    }
    if (loopshapingDefinition_->getInputFilter_r().getNumOutputs() > 0) {
      loopshapingConstraintImplementation_.reset(new LoopshapingConstraintOutputPattern<FULL_STATE_DIM,
                                                                                       FULL_INPUT_DIM,
                                                                                       SYSTEM_STATE_DIM,
                                                                                       SYSTEM_INPUT_DIM,
                                                                                       FILTER_STATE_DIM,
                                                                                       FILTER_INPUT_DIM,
                                                                                       NullLogicRules>(systemConstraint_,
                                                                                                       loopshapingDefinition_));
    }
  };

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
      systemConstraint_ = std::shared_ptr<SYSTEM_CONSTRAINT>(obj.systemConstraint_->clone());
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

    filter_state_vector_t x_filter_;
    filter_input_vector_t u_filter_;
    system_state_vector_t x_system_;
    system_input_vector_t u_system_;
    loopshapingDefinition_->getSystemState(x, x_system_);
    loopshapingDefinition_->getSystemInput(x, u, u_system_);
    loopshapingDefinition_->getFilterState(x, x_filter_);
    loopshapingDefinition_->getFilteredInput(x, u, u_filter_);

    if (systemConstraint_) {
      systemConstraint_->setCurrentStateAndControl(t, x_system_, u_system_);
    }

    auto &s_filter = loopshapingDefinition_->getInputFilter_s();
    if (s_filter.getNumOutputs() > 0 && !loopshapingDefinition_->eliminateInputs) {
      filter_constraint_dim = s_filter.getNumOutputs();
    } else {
      filter_constraint_dim = 0;
    }

    if (s_filter.getNumOutputs() > 0) {
      loopshapingConstraintImplementation_->setCurrentStateAndControl(t, x_system_, u_system_, x_filter_, u_filter_);
    }
  }

  virtual void getConstraint1(constraint1_vector_t &e) override {
    size_t numSystemConstraints = 0;
    if (systemConstraint_) {
      numSystemConstraints = systemConstraint_->numStateInputConstraint(BASE::t_);
      system_contraint1_vector_t e_system;
      systemConstraint_->getConstraint1(e_system);
      e.template segment<system_input_dim>(0) = e_system;
    }

    if (filter_constraint_dim > 0) {
      loopshapingConstraintImplementation_->getConstraint1(numSystemConstraints, e);
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
    size_t numSystemConstraints = 0;
    if (systemConstraint_) {
      numSystemConstraints = systemConstraint_->numStateInputConstraint(BASE::t_);

      system_constraint1_state_matrix_t C_system;
      systemConstraint_->getConstraint1DerivativesState(C_system);
      C.block(0, 0, numSystemConstraints, SYSTEM_STATE_DIM) = C_system.block(0, 0, numSystemConstraints, SYSTEM_STATE_DIM);
      if (FULL_STATE_DIM > SYSTEM_STATE_DIM) {
        C.block(0, SYSTEM_STATE_DIM, numSystemConstraints, FULL_STATE_DIM - SYSTEM_STATE_DIM).setZero();
      }
    }

    loopshapingConstraintImplementation_->getConstraint1DerivativesState(numSystemConstraints, C);
  }

  virtual void getConstraint1DerivativesControl(constraint1_input_matrix_t &D) override {
    size_t numSystemConstraints = 0;
    if (systemConstraint_) {
      numSystemConstraints = systemConstraint_->numStateInputConstraint(BASE::t_);
      system_constraint1_input_matrix_t D_system;
      systemConstraint_->getConstraint1DerivativesControl(D_system);
      D.block(0, 0, numSystemConstraints, SYSTEM_INPUT_DIM) = D_system.block(0, 0, numSystemConstraints, SYSTEM_INPUT_DIM);
      if (FULL_INPUT_DIM > SYSTEM_INPUT_DIM) {
        D.block(0, SYSTEM_INPUT_DIM, numSystemConstraints, FULL_INPUT_DIM - SYSTEM_INPUT_DIM).setZero();
      }
    }

    loopshapingConstraintImplementation_->getConstraint1DerivativesControl(numSystemConstraints, D);
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
    loopshapingConstraintImplementation_->getInequalityConstraintDerivativesState(dhdx);
  }

  virtual void getInequalityConstraintDerivativesInput(input_vector_array_t &dhdu) override {
    loopshapingConstraintImplementation_->getInequalityConstraintDerivativesInput(dhdu);
  }

  virtual void getInequalityConstraintSecondDerivativesState(state_matrix_array_t &ddhdxdx) override {
    loopshapingConstraintImplementation_->getInequalityConstraintSecondDerivativesState(ddhdxdx);
  }

  virtual void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t &ddhdudu) override {
    loopshapingConstraintImplementation_->getInequalityConstraintSecondDerivativesInput(ddhdudu);
  }

  virtual void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t &ddhdudx) override {
    loopshapingConstraintImplementation_->getInequalityConstraintDerivativesInputState(ddhdudx);
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
  std::shared_ptr<SYSTEM_CONSTRAINT> systemConstraint_;
  std::unique_ptr<LoopshapingConstraintImplementation> loopshapingConstraintImplementation_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};
} // namespace ocs2;

#endif //OCS2_LOOPSHAPINGCONSTRAINT_H
