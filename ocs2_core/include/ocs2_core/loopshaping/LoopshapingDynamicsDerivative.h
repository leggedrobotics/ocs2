//
// Created by ruben on 14.09.18.
//

#ifndef OCS2_LOOPSHAPINGDYNAMICSDERIVATIVE_H
#define OCS2_LOOPSHAPINGDYNAMICSDERIVATIVE_H

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/dynamics/DerivativesBase.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/logic/machine/LogicRulesMachine.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"

namespace ocs2 {
    template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
        size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
        size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
        class LOGIC_RULES_T=NullLogicRules>
    class LoopshapingDynamicsDerivative final : public DerivativesBase<FULL_STATE_DIM, FULL_INPUT_DIM, LOGIC_RULES_T>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using Ptr = std::shared_ptr<LoopshapingDynamicsDerivative>;

        using BASE = DerivativesBase<FULL_STATE_DIM, FULL_INPUT_DIM, LOGIC_RULES_T>;
        using scalar_t = typename BASE::scalar_t;
        using state_vector_t = typename BASE::state_vector_t;
        using input_vector_t = typename BASE::input_vector_t;
        using state_matrix_t = typename BASE::state_matrix_t;
        using state_input_matrix_t = typename BASE::state_input_matrix_t;
        using dynamic_vector_t = typename BASE::dynamic_vector_t;
        using dynamic_state_matrix_t = typename BASE::dynamic_state_matrix_t;
        using dynamic_input_matrix_t = typename BASE::dynamic_input_matrix_t;

        using SYSTEM_DERIVATIVE = DerivativesBase<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, LOGIC_RULES_T>;
        static constexpr size_t system_state_dim = SYSTEM_STATE_DIM;
        static constexpr size_t system_input_dim = SYSTEM_INPUT_DIM;
        using system_state_vector_t = typename SYSTEM_DERIVATIVE::state_vector_t;
        using system_input_vector_t = typename SYSTEM_DERIVATIVE::input_vector_t;
        using system_state_matrix_t = typename SYSTEM_DERIVATIVE::state_matrix_t;
        using system_state_input_matrix_t = typename SYSTEM_DERIVATIVE::state_input_matrix_t;
        using system_dynamic_state_matrix_t = typename SYSTEM_DERIVATIVE::dynamic_state_matrix_t;
        using system_dynamic_input_matrix_t = typename SYSTEM_DERIVATIVE::dynamic_input_matrix_t;

        static constexpr size_t filter_state_dim = FILTER_STATE_DIM;
        static constexpr size_t filter_input_dim = FILTER_INPUT_DIM;
        using filter_state_vector_t = Eigen::Matrix<scalar_t, filter_state_dim, 1>;
        using filter_input_vector_t = Eigen::Matrix<scalar_t, filter_input_dim, 1>;

        LoopshapingDynamicsDerivative(const SYSTEM_DERIVATIVE& systemDerivative, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) :
            BASE(),
            systemDerivative_(systemDerivative.clone()),
            loopshapingDefinition_(loopshapingDefinition) { };

        ~LoopshapingDynamicsDerivative() override = default;

        LoopshapingDynamicsDerivative(const LoopshapingDynamicsDerivative& obj) :
            BASE(),
            systemDerivative_(obj.systemDerivative_->clone()),
            loopshapingDefinition_(obj.loopshapingDefinition_)
        {}

        LoopshapingDynamicsDerivative* clone() const override {
          return new LoopshapingDynamicsDerivative(*this);
        };

        void initializeModel(
            LogicRulesMachine<LOGIC_RULES_T>& logicRulesMachine,
            const size_t& partitionIndex,
            const char* algorithmName=NULL) override
        {
          BASE::initializeModel(logicRulesMachine, partitionIndex, algorithmName);
          systemDerivative_->initializeModel(logicRulesMachine, partitionIndex, algorithmName);
        }

        void setCurrentStateAndControl(
            const scalar_t& t,
            const state_vector_t& x,
            const input_vector_t& u) override
        {
          BASE::setCurrentStateAndControl(t, x, u);

          system_state_vector_t systemstate;
          system_input_vector_t systeminput;
          loopshapingDefinition_->getSystemState(x, systemstate);
          loopshapingDefinition_->getSystemInput(x, u, systeminput);
          systemDerivative_->setCurrentStateAndControl(t, systemstate, systeminput);
        }

        void getFlowMapDerivativeTime(state_vector_t& df) override
        {
          system_state_vector_t system_df;
          filter_state_vector_t filter_df;
          systemDerivative_->getFlowMapDerivativeTime(system_df);
          filter_df.setZero();
          loopshapingDefinition_->concatenateSystemAndFilterState(system_df, filter_df, df);
        }

        void getFlowMapDerivativeState(state_matrix_t& A) override
        {
          // system
          system_state_matrix_t A_system;
          systemDerivative_->getFlowMapDerivativeState(A_system);
          A.template block<system_state_dim, system_state_dim>(0, 0) = A_system;

          // filters
          const auto& r_filter = loopshapingDefinition_->getInputFilter_r();
          const auto& s_filter = loopshapingDefinition_->getInputFilter_s();

          if (r_filter.getNumStates() > 0){
            A.template block(system_state_dim,
                             system_state_dim,
                             r_filter.getNumStates(),
                             r_filter.getNumStates()) =
                r_filter.getA();
            A.template block(0, system_state_dim, system_state_dim, r_filter.getNumStates()).setZero();
            A.template block(system_state_dim, 0, r_filter.getNumStates(), system_state_dim).setZero();
          }

          if (s_filter.getNumStates() > 0){
            A.template block(system_state_dim,
                             system_state_dim,
                             s_filter.getNumStates(),
                             s_filter.getNumStates()) =
                s_filter.getA();
            A.template block(system_state_dim, 0, s_filter.getNumStates(), system_state_dim).setZero();
            if (loopshapingDefinition_->eliminateInputs){
              system_state_input_matrix_t B_system;
              systemDerivative_->getFlowMapDerivativeInput(B_system);
              A.template block(0, system_state_dim, system_state_dim, s_filter.getNumStates()) =
                  B_system * s_filter.getC();
            } else{
              A.template block(0, system_state_dim, system_state_dim, s_filter.getNumStates()).setZero();
            }
          }
        };

        void getFlowMapDerivativeInput(state_input_matrix_t& B) override
        {
          // system
          system_state_input_matrix_t B_system;
          systemDerivative_->getFlowMapDerivativeInput(B_system);

          // filter
          const auto& r_filter = loopshapingDefinition_->getInputFilter_r();
          const auto& s_filter = loopshapingDefinition_->getInputFilter_s();

          if (r_filter.getNumStates() > 0){
            B.template block(0, 0, system_state_dim, system_input_dim) = B_system;
            B.template block(system_state_dim,
                             0,
                             r_filter.getNumStates(),
                             r_filter.getNumInputs()) =
                r_filter.getB();
          }

          if (s_filter.getNumStates() > 0){
            if (loopshapingDefinition_->eliminateInputs){
              B.template block(0, 0, system_state_dim, s_filter.getNumInputs()) = B_system * s_filter.getD();
              B.template block(system_state_dim, 0,
                               s_filter.getNumStates(),
                               s_filter.getNumInputs()) = s_filter.getB();
            } else {
              B.template block(0, 0, system_state_dim, system_input_dim) = B_system;
              B.template block(system_state_dim,
                               system_input_dim,
                               s_filter.getNumStates(),
                               s_filter.getNumInputs()) = s_filter.getB();
              B.template block(0, system_input_dim, system_state_dim, s_filter.getNumInputs()).setZero();
              B.template block(system_state_dim, 0, s_filter.getNumStates(), system_input_dim).setZero();
            }
          }
        };

        void getJumpMapDerivativeTime(state_vector_t& dg) {
          system_state_vector_t system_dg;
          filter_state_vector_t filter_dg;
          systemDerivative_->getJumpMapDerivativeTime(system_dg);
          filter_dg.setZero();
          loopshapingDefinition_->concatenateSystemAndFilterState(system_dg, filter_dg, dg);
        }

        void getJumpMapDerivativeState(state_matrix_t& G) override {
          // system
          system_state_matrix_t G_system;
          systemDerivative_->getJumpMapDerivativeState(G_system);
          G.template block<system_state_dim, system_state_dim>(0, 0) = G_system;

          // filter
          G.template block<filter_state_dim, filter_state_dim>(system_state_dim, system_state_dim).setIdentity();

          // Off diagonal coupling blocks
          G.template block<system_state_dim, filter_state_dim>(0, system_state_dim).setZero();
          G.template block<filter_state_dim, system_state_dim>(system_state_dim, 0).setZero();
        }

        void getJumpMapDerivativeInput(state_input_matrix_t& H) override {
          // system
          system_state_input_matrix_t H_system;
          systemDerivative_->getJumpMapDerivativeInput(H_system);

          const auto& r_filter = loopshapingDefinition_->getInputFilter_r();
          const auto& s_filter = loopshapingDefinition_->getInputFilter_s();

          if (r_filter.getNumStates() > 0){
            H.template block(0, 0, system_state_dim, system_input_dim) = H_system;
            H.template block(system_state_dim, 0, s_filter.getNumStates(), system_input_dim).setZero();
          }

          if (s_filter.getNumStates() > 0){
            if (loopshapingDefinition_->eliminateInputs){
              H.template block(0, 0, system_state_dim, s_filter.getNumInputs()) = H_system * s_filter.getD();
              H.template block(system_state_dim, 0, s_filter.getNumStates(), s_filter.getNumInputs()).setZero();
            } else {
              H.template block(0, 0, system_state_dim, system_input_dim) = H_system;
              H.template block(system_state_dim, system_input_dim, s_filter.getNumStates(), s_filter.getNumInputs()).setZero();
              H.template block(0, system_input_dim, system_state_dim, s_filter.getNumInputs()).setZero();
              H.template block(system_state_dim, 0, s_filter.getNumStates(), system_input_dim).setZero();
            }
          }
        }

        void getGuardSurfacesDerivativeTime(dynamic_vector_t& D_t_gamma) override {
          systemDerivative_->getGuardSurfacesDerivativeTime(D_t_gamma);
        }

        void getGuardSurfacesDerivativeState(dynamic_state_matrix_t& D_x_gamma) override {
          system_dynamic_state_matrix_t D_x_gamma_system;
          systemDerivative_->getGuardSurfacesDerivativeState(D_x_gamma_system);
          D_x_gamma.template block(0, 0, D_x_gamma.rows(), system_state_dim) = D_x_gamma_system;
          D_x_gamma.template block(0, system_state_dim, D_x_gamma.rows(), filter_state_dim).setZero();
        }

        void getGuardSurfacesDerivativeInput(dynamic_input_matrix_t& D_u_gamma) override {
          D_u_gamma.setZero();
          system_dynamic_input_matrix_t D_u_gamma_system;
          systemDerivative_->getGuardSurfacesDerivativeInput(D_u_gamma_system);

          const auto& r_filter = loopshapingDefinition_->getInputFilter_r();
          const auto& s_filter = loopshapingDefinition_->getInputFilter_s();

          if (r_filter.getNumStates() > 0){
            D_u_gamma.template block(0, 0, D_u_gamma.rows(), system_input_dim) = D_u_gamma_system;
          }

          if (s_filter.getNumStates() > 0){
            if (loopshapingDefinition_->eliminateInputs){
              D_u_gamma.template block(0, 0, D_u_gamma.rows(), s_filter.getNumInputs()) = D_u_gamma_system * s_filter.getD();
            } else {
              D_u_gamma.template block(0, 0, D_u_gamma.rows(), system_input_dim) = D_u_gamma_system;
              D_u_gamma.template block(0, system_input_dim, D_u_gamma.rows(), filter_input_dim).setZero();
            }
          }
        }

    private:
        std::unique_ptr<SYSTEM_DERIVATIVE> systemDerivative_;
        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
    };

}

#endif //OCS2_LOOPSHAPINGDYNAMICSDERIVATIVE_H
