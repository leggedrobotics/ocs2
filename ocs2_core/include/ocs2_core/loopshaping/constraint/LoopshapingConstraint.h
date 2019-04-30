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

  using SYSTEM_CONSTRAINT = ConstraintBase<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, LOGIC_RULES_T>;

  using FULL_DIMENSIONS = ocs2::Dimensions<FULL_STATE_DIM, FULL_INPUT_DIM>;
  using scalar_t = typename FULL_DIMENSIONS::scalar_t;
  using state_vector_t = typename FULL_DIMENSIONS::state_vector_t;
  using input_vector_t = typename FULL_DIMENSIONS::input_vector_t;
  using constraint1_vector_t = typename FULL_DIMENSIONS::constraint1_vector_t;
  using constraint1_state_matrix_t = typename FULL_DIMENSIONS::constraint1_state_matrix_t;
  using constraint1_input_matrix_t = typename FULL_DIMENSIONS::constraint1_input_matrix_t;
  using constraint2_vector_t = typename FULL_DIMENSIONS::constraint2_vector_t;
  using constraint2_state_matrix_t = typename FULL_DIMENSIONS::constraint2_state_matrix_t;
  using scalar_array_t = typename FULL_DIMENSIONS::scalar_array_t;
  using state_vector_array_t = typename FULL_DIMENSIONS::state_vector_array_t;
  using input_vector_array_t = typename FULL_DIMENSIONS::input_vector_array_t;
  using state_matrix_array_t = typename FULL_DIMENSIONS::state_matrix_array_t;
  using input_matrix_array_t = typename FULL_DIMENSIONS::input_matrix_array_t;
  using input_state_matrix_array_t = typename FULL_DIMENSIONS::input_state_matrix_array_t;

  using SYSTEM_DIMENSIONS = ocs2::Dimensions<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM>;
  using system_state_vector_t = typename SYSTEM_DIMENSIONS::state_vector_t;
  using system_input_vector_t = typename SYSTEM_DIMENSIONS::input_vector_t;
  using system_contraint1_vector_t = typename SYSTEM_DIMENSIONS::constraint1_vector_t;
  using system_contraint2_vector_t = typename SYSTEM_DIMENSIONS::constraint2_vector_t;
  using system_constraint1_state_matrix_t = typename SYSTEM_DIMENSIONS::constraint1_state_matrix_t;
  using system_constraint1_input_matrix_t = typename SYSTEM_DIMENSIONS::constraint1_input_matrix_t;
  using system_constraint2_state_matrix_t = typename SYSTEM_DIMENSIONS::constraint2_state_matrix_t;

  using FILTER_DIMENSIONS = ocs2::Dimensions<FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using filter_state_vector_t = typename FILTER_DIMENSIONS::state_vector_t;
  using filter_input_vector_t = typename FILTER_DIMENSIONS::input_vector_t;

  using LoopshapingConstraintImplementation = LoopshapingConstraintImplementationBase<FULL_STATE_DIM,
                                                                                      FULL_INPUT_DIM,
                                                                                      SYSTEM_STATE_DIM,
                                                                                      SYSTEM_INPUT_DIM,
                                                                                      FILTER_STATE_DIM,
                                                                                      FILTER_INPUT_DIM,
                                                                                      LOGIC_RULES_T>;

  LoopshapingConstraint(const SYSTEM_CONSTRAINT &systemConstraint,
                        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) :
      BASE(),
      systemConstraint_(systemConstraint.clone()),
      loopshapingDefinition_(std::move(loopshapingDefinition)) {
    loadImplementation();
  };

  LoopshapingConstraint(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) :
      BASE(),
      systemConstraint_(nullptr),
      loopshapingDefinition_(std::move(loopshapingDefinition)) {
    loadImplementation();
  };

  ~LoopshapingConstraint() override = default;

  LoopshapingConstraint(const LoopshapingConstraint &obj) :
      BASE(),
      systemConstraint_(nullptr),
      loopshapingDefinition_(obj.loopshapingDefinition_) {
    if (obj.systemConstraint_) {
      systemConstraint_ = std::shared_ptr<SYSTEM_CONSTRAINT>(obj.systemConstraint_->clone());
    };
    loadImplementation();
  }

  LoopshapingConstraint *clone() const override {
    return new LoopshapingConstraint(*this);
  };

  void initializeModel(
      LogicRulesMachine<LOGIC_RULES_T> &logicRulesMachine,
      const size_t &partitionIndex,
      const char *algorithmName = NULL) override {
    BASE::initializeModel(logicRulesMachine, partitionIndex, algorithmName);
    if (systemConstraint_) {
      systemConstraint_->initializeModel(logicRulesMachine, partitionIndex, algorithmName);
    };
    loadImplementation();
  }

  void setCurrentStateAndControl(
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

    loopshapingConstraintImplementation_->setCurrentStateAndControl(t, x_system_, u_system_, x_filter_, u_filter_);
  }

  void getConstraint1(constraint1_vector_t &e) override {
    size_t numSystemConstraints = 0;
    if (systemConstraint_) {
      numSystemConstraints = systemConstraint_->numStateInputConstraint(BASE::t_);
      system_contraint1_vector_t e_system;
      systemConstraint_->getConstraint1(e_system);
      e.template segment<SYSTEM_INPUT_DIM>(0) = e_system;
    }

    loopshapingConstraintImplementation_->getConstraint1(numSystemConstraints, e);
  }

  size_t numStateInputConstraint(const scalar_t &time) override {
    size_t numSystemConstraints = 0;
    if (systemConstraint_) {
      numSystemConstraints = systemConstraint_->numStateInputConstraint(time);
    }
    return loopshapingConstraintImplementation_->numStateInputConstraint(numSystemConstraints, time);
  }

  void getConstraint2(constraint2_vector_t &h) override {
    if (systemConstraint_) {
      system_contraint2_vector_t h_system;
      systemConstraint_->getConstraint2(h_system);
      h.template segment<SYSTEM_INPUT_DIM>(0) = h_system;
    }
  }

  size_t numStateOnlyConstraint(const scalar_t &time) override {
    if (systemConstraint_) {
      return systemConstraint_->numStateOnlyConstraint(time);
    } else {
      return 0;
    }
  }

  void getInequalityConstraint(scalar_array_t &h) override {
    if (systemConstraint_) {
      systemConstraint_->getInequalityConstraint(h);
    }
  }

  size_t numInequalityConstraint(const scalar_t &time) override {
    if (systemConstraint_) {
      return systemConstraint_->numInequalityConstraint(time);
    } else {
      return 0;
    }
  }

  void getFinalConstraint2(constraint2_vector_t &h_f) override {
    if (systemConstraint_) {
      system_contraint2_vector_t h_f_system;
      systemConstraint_->getFinalConstraint2(h_f_system);
      h_f.template segment<SYSTEM_INPUT_DIM>(0) = h_f_system;
    }
  }

  size_t numStateOnlyFinalConstraint(const scalar_t &time) override {
    if (systemConstraint_) {
      return systemConstraint_->numStateOnlyFinalConstraint(time);
    } else {
      return 0;
    }
  }

  void getConstraint1DerivativesState(constraint1_state_matrix_t &C) override {
    size_t numSystemConstraints = 0;
    if (systemConstraint_) {
      numSystemConstraints = systemConstraint_->numStateInputConstraint(BASE::t_);

      system_constraint1_state_matrix_t C_system;
      systemConstraint_->getConstraint1DerivativesState(C_system);
      C.block(0, 0, numSystemConstraints, SYSTEM_STATE_DIM) =
          C_system.block(0, 0, numSystemConstraints, SYSTEM_STATE_DIM);

      // Add zeros for extra states
      if (FULL_STATE_DIM > SYSTEM_STATE_DIM) {
        C.block(0, SYSTEM_STATE_DIM, numSystemConstraints, FULL_STATE_DIM - SYSTEM_STATE_DIM).setZero();
      }
    }

    loopshapingConstraintImplementation_->getConstraint1DerivativesState(numSystemConstraints, C);
  }

  void getConstraint1DerivativesControl(constraint1_input_matrix_t &D) override {
    size_t numSystemConstraints = 0;
    if (systemConstraint_) {
      numSystemConstraints = systemConstraint_->numStateInputConstraint(BASE::t_);
      system_constraint1_input_matrix_t D_system;
      systemConstraint_->getConstraint1DerivativesControl(D_system);
      D.block(0, 0, numSystemConstraints, SYSTEM_INPUT_DIM) =
          D_system.block(0, 0, numSystemConstraints, SYSTEM_INPUT_DIM);

      // Add zeros for extra inputs
      if (FULL_INPUT_DIM > SYSTEM_INPUT_DIM) {
        D.block(0, SYSTEM_INPUT_DIM, numSystemConstraints, FULL_INPUT_DIM - SYSTEM_INPUT_DIM).setZero();
      }
    }

    loopshapingConstraintImplementation_->getConstraint1DerivativesControl(numSystemConstraints, D);
  }

  void getConstraint2DerivativesState(constraint2_state_matrix_t &F) override {
    if (systemConstraint_) {
      system_constraint2_state_matrix_t F_system;
      systemConstraint_->getConstraint2DerivativesState(F_system);
      F.template block<SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM>(0, 0) = F_system;
      F.template block<SYSTEM_INPUT_DIM, FILTER_STATE_DIM>(0, SYSTEM_STATE_DIM).setZero();
    }
  }

  void getInequalityConstraintDerivativesState(state_vector_array_t &dhdx) override {
    loopshapingConstraintImplementation_->getInequalityConstraintDerivativesState(dhdx);
  }

  void getInequalityConstraintDerivativesInput(input_vector_array_t &dhdu) override {
    loopshapingConstraintImplementation_->getInequalityConstraintDerivativesInput(dhdu);
  }

  void getInequalityConstraintSecondDerivativesState(state_matrix_array_t &ddhdxdx) override {
    loopshapingConstraintImplementation_->getInequalityConstraintSecondDerivativesState(ddhdxdx);
  }

  void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t &ddhdudu) override {
    loopshapingConstraintImplementation_->getInequalityConstraintSecondDerivativesInput(ddhdudu);
  }

  void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t &ddhdudx) override {
    loopshapingConstraintImplementation_->getInequalityConstraintDerivativesInputState(ddhdudx);
  }

  void getFinalConstraint2DerivativesState(constraint2_state_matrix_t &F_f) {
    if (systemConstraint_) {
      system_constraint2_state_matrix_t F_f_system;
      systemConstraint_->getFinalConstraint2DerivativesState(F_f_system);
      F_f.template block<SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM>(0, 0) = F_f_system;
      F_f.template block<SYSTEM_INPUT_DIM, FILTER_STATE_DIM>(0, SYSTEM_STATE_DIM).setZero();
    }
  }

 private:
  void loadImplementation() {
    if (loopshapingDefinition_->getInputFilter_s().getNumOutputs() > 0) {
      if (loopshapingDefinition_->eliminateInputs) {
        loopshapingConstraintImplementation_.reset(new LoopshapingConstraintEliminatePattern<FULL_STATE_DIM,
                                                                                             FULL_INPUT_DIM,
                                                                                             SYSTEM_STATE_DIM,
                                                                                             SYSTEM_INPUT_DIM,
                                                                                             FILTER_STATE_DIM,
                                                                                             FILTER_INPUT_DIM,
                                                                                             LOGIC_RULES_T>(
            systemConstraint_,
            loopshapingDefinition_));
      } else {
        loopshapingConstraintImplementation_.reset(new LoopshapingConstraintInputPattern<FULL_STATE_DIM,
                                                                                         FULL_INPUT_DIM,
                                                                                         SYSTEM_STATE_DIM,
                                                                                         SYSTEM_INPUT_DIM,
                                                                                         FILTER_STATE_DIM,
                                                                                         FILTER_INPUT_DIM,
                                                                                         LOGIC_RULES_T>(
            systemConstraint_,
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
                                                                                        LOGIC_RULES_T>(
          systemConstraint_,
          loopshapingDefinition_));
    }
  }

  std::shared_ptr<SYSTEM_CONSTRAINT> systemConstraint_;
  std::unique_ptr<LoopshapingConstraintImplementation> loopshapingConstraintImplementation_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};
} // namespace ocs2;

#endif //OCS2_LOOPSHAPINGCONSTRAINT_H
