//
// Created by ruben on 12.07.18.
//

#ifndef OCS2_COMKINODYNAMICSAUGMENTED_H
#define OCS2_COMKINODYNAMICSAUGMENTED_H

#include "ComKinoDynamicsBase.h"
#include "filterDynamics.h"

namespace switched_model {

    template<size_t JOINT_COORD_SIZE, size_t INPUTFILTER_STATE_SIZE, size_t INPUTFILTER_INPUT_SIZE>
    class ComKinoDynamicsAugmented : public ocs2::ControlledSystemBase<
        12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
        12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE,
        SwitchedModelPlannerLogicRules<12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE>>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using ComKinoDynamicsBase_t = ComKinoDynamicsBase<JOINT_COORD_SIZE>;
        using kinematic_model_t = ComKinoDynamicsBase_t::kinematic_model_t;
        using com_model_t = ComKinoDynamicsBase_t::com_model_t;
        using ground_profile_ptr_t = ComKinoDynamicsBase_t::ground_profile_ptr_t;
        using flat_ground_profile_t = ComKinoDynamicsBase_t::flat_ground_profile_t;
        using flat_ground_profile_ptr_t = ComKinoDynamicsBase_t::flat_ground_profile_ptr_t;
        using logic_rules_machine_t = ComKinoDynamicsBase_t::logic_rules_machine_t;

        using input_filter_t = FilterDynamics<INPUTFILTER_STATE_SIZE, INPUTFILTER_INPUT_SIZE, 12+JOINT_COORD_SIZE>;

        using BASE = ocs2::ControlledSystemBase<
            12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE,
            SwitchedModelPlannerLogicRules<12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE>>;
        using scalar_t = BASE::scalar_t ;
        using state_vector_t = BASE::state_vector_t;
        using input_vector_t = BASE::input_vector_t;
        using dynamic_vector_t = BASE::dynamic_vector_t;

        ComKinoDynamicsAugmented(const FilterSettings& filterSettings,
                                 const kinematic_model_t& kinematicModel,
                                 const com_model_t& comModel,
                                 const Model_Settings& options = Model_Settings(),
                                 const ground_profile_ptr_t& groundProfilePtr = flat_ground_profile_ptr_t(new flat_ground_profile_t()))
        : comKinoDynamics_(kinematicModel, comModel, options, groundProfilePtr),
          inputFilter_(filterSettings) {};

        ~ComKinoDynamicsAugmented() override = default;

        ComKinoDynamicsAugmented<JOINT_COORD_SIZE, INPUTFILTER_STATE_SIZE, INPUTFILTER_INPUT_SIZE>* clone() const override
        {
            return new ComKinoDynamicsAugmented<JOINT_COORD_SIZE, INPUTFILTER_STATE_SIZE, INPUTFILTER_INPUT_SIZE>(*this);
        };

        virtual void initializeModel(logic_rules_machine_t& logicRulesMachine,
                                     const size_t& partitionIndex, const char* algorithmName=NULL) override
        {
            comKinoDynamics_.initializeModel(logicRulesMachine, partitionIndex, algorithmName);
        }

        void computeFlowMap(const scalar_t& t, const state_vector_t& x, const input_vector_t& u, state_vector_t& dxdt) override
        {
            // ComKinoDynamics = first 12 + JOINT_COORD_SIZE rows
            ComKinoDynamicsBase_t::state_vector_t x_comKinDynamics = x.template segment<12+JOINT_COORD_SIZE>(0);
            ComKinoDynamicsBase_t::input_vector_t u_comKinDynamics = u.template segment<12+JOINT_COORD_SIZE>(0);
            ComKinoDynamicsBase_t::state_vector_t dxdt_comKinDynamics;

            comKinoDynamics_.computeFlowMap(t, x_comKinDynamics, u_comKinDynamics, dxdt_comKinDynamics);
            dxdt.template segment<12+JOINT_COORD_SIZE>(0) = dxdt_comKinDynamics;

            // input filter dynamics
            input_filter_t::state_vector_t x_inputFilter = x.template segment<INPUTFILTER_STATE_SIZE>(12+JOINT_COORD_SIZE);
            input_filter_t::input_vector_t u_inputFilter = u.template segment<INPUTFILTER_INPUT_SIZE>(12+JOINT_COORD_SIZE);
            dxdt.template segment<INPUTFILTER_STATE_SIZE>(12+JOINT_COORD_SIZE) =
                inputFilter_.getA()*x_inputFilter + inputFilter_.getB()*u_inputFilter;
        };

        void computeJumpMap(const scalar_t& time, const state_vector_t& state, state_vector_t& mappedState) override {
          mappedState = state;
        }

        void computeGuardSurfaces(const scalar_t& t, const state_vector_t& x, dynamic_vector_t& guardSurfacesValue) override {
            ComKinoDynamicsBase_t::state_vector_t x_comKinDynamics = x.template segment<12+JOINT_COORD_SIZE>(0);
            comKinoDynamics_.computeGuardSurfaces(t, x_comKinDynamics, guardSurfacesValue);
        };

    private:
        ComKinoDynamicsBase_t comKinoDynamics_;
        input_filter_t inputFilter_;
    };

}; // namespace switched_model

#endif //OCS2_COMKINODYNAMICSAUGMENTED_H
