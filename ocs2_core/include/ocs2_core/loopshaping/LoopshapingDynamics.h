//
// Created by ruben on 14.09.18.
//

#ifndef OCS2_LOOPSHAPINGDYNAMICS_H
#define OCS2_LOOPSHAPINGDYNAMICS_H

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/dynamics/ControlledSystemBase.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/logic/machine/LogicRulesMachine.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"

namespace ocs2 {
    template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
        size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
        size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
        class LOGIC_RULES_T=NullLogicRules>
    class LoopshapingDynamics final : public
                                ControlledSystemBase<FULL_STATE_DIM, FULL_INPUT_DIM, LOGIC_RULES_T>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using Ptr = std::shared_ptr<LoopshapingDynamics>;

        using BASE = ControlledSystemBase<FULL_STATE_DIM, FULL_INPUT_DIM, LOGIC_RULES_T>;
        using scalar_t = typename BASE::scalar_t;
        using state_vector_t = typename BASE::state_vector_t;
        using input_vector_t = typename BASE::input_vector_t;
        using dynamic_vector_t = typename BASE::dynamic_vector_t;

        using SYSTEM = ocs2::ControlledSystemBase<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, LOGIC_RULES_T>;
        static constexpr size_t system_state_dim = SYSTEM_STATE_DIM;
        static constexpr size_t system_input_dim = SYSTEM_INPUT_DIM;
        using system_state_vector_t = typename SYSTEM::state_vector_t;
        using system_input_vector_t = typename SYSTEM::input_vector_t;

        static constexpr size_t filter_state_dim = FILTER_STATE_DIM;
        static constexpr size_t filter_input_dim = FILTER_INPUT_DIM;
        using filter_state_vector_t = Eigen::Matrix<scalar_t, filter_state_dim, 1>;
        using filter_input_vector_t = Eigen::Matrix<scalar_t, filter_input_dim, 1>;

        LoopshapingDynamics(const SYSTEM& controlledSystem, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) :
            BASE(),
            controlledSystem_(controlledSystem.clone()),
            loopshapingDefinition_(loopshapingDefinition) { }

        ~LoopshapingDynamics() override = default;

        LoopshapingDynamics(const LoopshapingDynamics& obj) :
            BASE(),
            controlledSystem_(obj.controlledSystem_->clone()),
            loopshapingDefinition_(obj.loopshapingDefinition_)
        {}

        LoopshapingDynamics* clone() const override {
          return new LoopshapingDynamics(*this);
        };

        void initializeModel(
            LogicRulesMachine<LOGIC_RULES_T>& logicRulesMachine,
            const size_t& partitionIndex,
            const char* algorithmName=NULL) override
        {
          BASE::initializeModel(logicRulesMachine, partitionIndex, algorithmName);
          controlledSystem_->initializeModel(logicRulesMachine, partitionIndex, algorithmName);
        }

        void computeFlowMap(
            const scalar_t& time,
            const state_vector_t& state,
            const input_vector_t& input,
            state_vector_t& stateDerivative) override
        {
            system_state_vector_t systemstate, systemstateDerivative;
            filter_state_vector_t filterstate, filterstateDerivative;
            system_input_vector_t systeminput;
            filter_input_vector_t filteredinput;

            loopshapingDefinition_->getSystemState(state, systemstate);
            loopshapingDefinition_->getSystemInput(state, input, systeminput);
            loopshapingDefinition_->getFilterState(state, filterstate);
            loopshapingDefinition_->getFilteredInput(state, input, filteredinput);

            controlledSystem_->computeFlowMap(time, systemstate, systeminput, systemstateDerivative);

            const auto& r_filter = loopshapingDefinition_->getInputFilter_r();
            const auto& s_filter = loopshapingDefinition_->getInputFilter_s();

            if (r_filter.getNumOutputs() > 0){
              filterstateDerivative = r_filter.getA() * filterstate + r_filter.getB() * systeminput;
            }

            if (s_filter.getNumOutputs() > 0){
              filterstateDerivative = s_filter.getA() * filterstate + s_filter.getB() * filteredinput;
            }

            loopshapingDefinition_->concatenateSystemAndFilterState(systemstateDerivative, filterstateDerivative, stateDerivative);
        }

        virtual void computeJumpMap(
            const scalar_t& time,
            const state_vector_t& state,
            state_vector_t& mappedState) override
        {
            system_state_vector_t systemstate, systemMappedState;
            filter_state_vector_t filterstate, filterMappedState;
            loopshapingDefinition_->getSystemState(state, systemstate);
            loopshapingDefinition_->getFilterState(state, filterstate);

            controlledSystem_->computeJumpMap(time, systemstate, systemMappedState);
            filterMappedState = filterstate;

            loopshapingDefinition_->concatenateSystemAndFilterState(systemMappedState, filterMappedState, mappedState);
        }

        virtual void computeGuardSurfaces(
            const scalar_t& time,
            const state_vector_t& state,
            dynamic_vector_t& guardSurfacesValue) override
        {
            system_state_vector_t systemstate;
            loopshapingDefinition_->getSystemState(state, systemstate);
            controlledSystem_->computeGuardSurfaces(time, systemstate, guardSurfacesValue);
        }

    private:
        std::unique_ptr<SYSTEM> controlledSystem_;
        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
    };
}



#endif //OCS2_LOOPSHAPINGDYNAMICS_H
