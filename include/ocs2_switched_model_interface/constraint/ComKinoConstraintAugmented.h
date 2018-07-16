//
// Created by ruben on 12.07.18.
//

#ifndef OCS2_COMKINOCONSTRAINTAUGMENTED_H
#define OCS2_COMKINOCONSTRAINTAUGMENTED_H

#include "ComKinoConstraintBase.h"
#include "../dynamics/filterDynamics.h"

namespace switched_model {

    template<size_t JOINT_COORD_SIZE, size_t INPUTFILTER_STATE_SIZE, size_t INPUTFILTER_INPUT_SIZE>
    class ComKinoConstraintAugmented : public ocs2::ConstraintBase<
        12 + JOINT_COORD_SIZE,
        12 + JOINT_COORD_SIZE,
        SwitchedModelPlannerLogicRules<
            JOINT_COORD_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE>> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using ComKinConstraintBase_t = ComKinoConstraintBase<JOINT_COORD_SIZE>;
        using kinematic_model_t = ComKinConstraintBase_t::kinematic_model_t;
        using com_model_t = ComKinConstraintBase_t::com_model_t;
        using contact_flag_t = ComKinConstraintBase_t::contact_flag_t;

        using logic_rules_machine_t = SwitchedModelPlannerLogicRules<
            JOINT_COORD_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE>;

        using input_filter_t = FilterDynamics<INPUTFILTER_STATE_SIZE, INPUTFILTER_INPUT_SIZE, 12+JOINT_COORD_SIZE>;

        using BASE = ocs2::ConstraintBase<
            12 + JOINT_COORD_SIZE,
            12 + JOINT_COORD_SIZE,
            SwitchedModelPlannerLogicRules<
                JOINT_COORD_SIZE,
                12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
                12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE>>;

        ComKinoConstraintAugmented(
            const FilterSettings &filterSettings,
            const kinematic_model_t& kinematicModel,
            const com_model_t& comModel,
            const Model_Settings& options = Model_Settings()) :
        comKinConstraintBase_(kinematicModel, comModel, options),
        inputFilter_(filterSettings) {};

        ~ComKinoConstraintAugmented() override = default;

        ComKinoConstraintAugmented<JOINT_COORD_SIZE, INPUTFILTER_STATE_SIZE, INPUTFILTER_INPUT_SIZE>* clone() const override
        {
          return ComKinoConstraintAugmented<JOINT_COORD_SIZE, INPUTFILTER_STATE_SIZE, INPUTFILTER_INPUT_SIZE>(*this);
        };

        void initializeModel(
            logic_rules_machine_t& logicRulesMachine,
            const size_t& partitionIndex,
            const char* algorithmName=NULL) override
        {
          comKinConstraintBase_.initializeModel();
        };

        void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override {
          ComKinConstraintBase_t::state_vector_t x_comKinoDynamics = x.template segment<12+JOINT_COORD_SIZE>(0);
          ComKinConstraintBase_t::input_vector_t u_comKinDynamics = u.template segment<12+JOINT_COORD_SIZE>(0);
          comKinConstraintBase_.setCurrentStateAndControl(t, x_comKinoDynamics, u_comKinDynamics);
        }

        void getConstraint1(constraint1_vector_t& e) override {
          ComKinConstraintBase_t::constraint1_vector_t e_comKinoDynamics;
          comKinConstraintBase_.getConstraint1(e_comKinoDynamics);
          e.template segment<12+JOINT_COORD_SIZE>(0) = e_comKinoDynamics;
          e.template segment<BASE::DIMENSIONS::MAX_CONSTRAINT1_DIM_ - 12+JOINT_COORD_SIZE>(12+JOINT_COORD_SIZE).setZero();
        }

        size_t numStateInputConstraint(const scalar_t& time) override;

        void getConstraint2(constraint2_vector_t& h) override;

        size_t numStateOnlyConstraint(const scalar_t& time) override;

        void getFinalConstraint2(constraint2_vector_t& h_f) override;

        size_t numStateOnlyFinalConstraint(const scalar_t& time) override;

        void getConstraint1DerivativesState(constraint1_state_matrix_t& C) override;

        void getConstraint1DerivativesControl(constraint1_input_matrix_t& D)  override;

        void getConstraint2DerivativesState(constraint2_state_matrix_t& F) override;

        void getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F_final) override;

        void setStanceLegs (const contact_flag_t& stanceLegs);

        void getStanceLegs (contact_flag_t& stanceLegs);

    private:
        ComKinConstraintBase_t comKinConstraintBase_;
        input_filter_t inputFilter_;
    }
}


#endif //OCS2_COMKINOCONSTRAINTAUGMENTED_H
