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
        12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
        12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE,
        SwitchedModelPlannerLogicRules<
            JOINT_COORD_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE>> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using ComKinConstraintBase_t = ComKinoConstraintBase<JOINT_COORD_SIZE,
            12+JOINT_COORD_SIZE,
            12+JOINT_COORD_SIZE,
            SwitchedModelPlannerLogicRules<
                JOINT_COORD_SIZE,
                12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
                12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE>>;
        using kinematic_model_t = typename ComKinConstraintBase_t::kinematic_model_t;
        using com_model_t = typename ComKinConstraintBase_t::com_model_t;
        using contact_flag_t = typename ComKinConstraintBase_t::contact_flag_t;

        using logic_rules_t = SwitchedModelPlannerLogicRules<
            JOINT_COORD_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE>;
        using logic_rules_machine_t = typename ocs2::LogicRulesMachine<12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE, logic_rules_t>;

        using input_filter_t = FilterDynamics<INPUTFILTER_STATE_SIZE, INPUTFILTER_INPUT_SIZE, 12+JOINT_COORD_SIZE>;

        using BASE = ocs2::ConstraintBase<
            12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE,
            SwitchedModelPlannerLogicRules<
                JOINT_COORD_SIZE,
                12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
                12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE>>;
        using scalar_t = typename BASE::scalar_t;
        using state_vector_t = typename BASE::state_vector_t;
        using input_vector_t = typename BASE::input_vector_t;
        using constraint1_vector_t = typename BASE::constraint1_vector_t;
        using constraint2_vector_t = typename BASE::constraint2_vector_t;
        using constraint1_state_matrix_t = typename BASE::constraint1_state_matrix_t;
        using constraint1_input_matrix_t = typename BASE::constraint1_input_matrix_t;
        using constraint2_state_matrix_t = typename BASE::constraint2_state_matrix_t;

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
          return new ComKinoConstraintAugmented<JOINT_COORD_SIZE, INPUTFILTER_STATE_SIZE, INPUTFILTER_INPUT_SIZE>(*this);
        };

        void initializeModel(
            logic_rules_machine_t& logicRulesMachine,
            const size_t& partitionIndex,
            const char* algorithmName=NULL) override
        {
          comKinConstraintBase_.initializeModel(logicRulesMachine, partitionIndex, algorithmName);
        };

        void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override {
          typename ComKinConstraintBase_t::state_vector_t x_comKinoDynamics = x.template segment<12+JOINT_COORD_SIZE>(0);
          typename ComKinConstraintBase_t::input_vector_t u_comKinDynamics = u.template segment<12+JOINT_COORD_SIZE>(0);
          comKinConstraintBase_.setCurrentStateAndControl(t, x_comKinoDynamics, u_comKinDynamics);
        }

        /**
         *  Since constraint1 of comKinoDynamics is time dependent, but time is not known in this function,
         *  Put the filter first, since it is of constant size.
        */
        void getConstraint1(constraint1_vector_t& e) override {
          typename ComKinConstraintBase_t::constraint1_vector_t e_comKinoDynamics;
          comKinConstraintBase_.getConstraint1(e_comKinoDynamics);
          e.template segment<12+JOINT_COORD_SIZE>(0).setZero();
          e.template segment<12+JOINT_COORD_SIZE>(12+JOINT_COORD_SIZE) = e_comKinoDynamics;
        }

        size_t numStateInputConstraint(const scalar_t& time) override {
          return inputFilter_.getNumOutputs() + comKinConstraintBase_.numStateInputConstraint(time);
        }

        /**
        *  Filter doesn't have state only constraints, so the comKinoDynamics constraints start at 0 directly.
        */
        void getConstraint2(constraint2_vector_t& h) override {
          typename ComKinConstraintBase_t::constraint2_vector_t h_comKinoDynamics;
          comKinConstraintBase_.getConstraint2(h_comKinoDynamics);
          h.template segment<12+JOINT_COORD_SIZE>(0) = h_comKinoDynamics;
        }

        size_t numStateOnlyConstraint(const scalar_t& time) override {
          return comKinConstraintBase_.numStateOnlyConstraint(time);
        }

        void getFinalConstraint2(constraint2_vector_t& h_f) override {
          typename ComKinConstraintBase_t::constraint2_vector_t h_f_comKinoDynamics;
          comKinConstraintBase_.getFinalConstraint2(h_f_comKinoDynamics);
          h_f.template segment<12+JOINT_COORD_SIZE>(0) = h_f_comKinoDynamics;
        }

        size_t numStateOnlyFinalConstraint(const scalar_t& time) override {
          comKinConstraintBase_.numStateOnlyFinalConstraint(time);
        }

        /*
         *  C = [0        C_inputfilter;
         *       C_kino   0]
         *    = [12+joint x 12+joint,  12+joint x filterstate
         *       12+joint x 12+joint,  12+joint x filterstate
         */
        void getConstraint1DerivativesState(constraint1_state_matrix_t& C) override {
          typename ComKinConstraintBase_t::constraint1_state_matrix_t C_comKinoDynamics;
          comKinConstraintBase_.getConstraint1DerivativesState(C_comKinoDynamics);
          C.template block<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE>(0, 0).setZero();
          C.template block<12+JOINT_COORD_SIZE, INPUTFILTER_STATE_SIZE>(0, 12+JOINT_COORD_SIZE) = inputFilter_.getC();
          C.template block<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE>(12+JOINT_COORD_SIZE, 0) = C_comKinoDynamics;
          C.template block<12+JOINT_COORD_SIZE, INPUTFILTER_STATE_SIZE>(12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE).setZero();
        }

        /*
         *  D = [-I      D_inputfilter
         *       D_kino  0]
         *    = [12+joint x 12+joint, 12+joint x filterinput
         *       12+joint x 12+joint, 12+joint x filterinput
         */
        void getConstraint1DerivativesControl(constraint1_input_matrix_t& D)  override {
          typename ComKinConstraintBase_t::constraint1_input_matrix_t D_comKinoDynamics;
          comKinConstraintBase_.getConstraint1DerivativesControl(D_comKinoDynamics);
          D.template block<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE>(0, 0) = -Eigen::Matrix<scalar_t, 12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE>::Identity();
          D.template block<12+JOINT_COORD_SIZE, INPUTFILTER_INPUT_SIZE>(0, 12+JOINT_COORD_SIZE) = inputFilter_.getD();
          D.template block<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE>(12+JOINT_COORD_SIZE, 0) = D_comKinoDynamics;
          D.template block<12+JOINT_COORD_SIZE, INPUTFILTER_INPUT_SIZE>(12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE).setZero();
        }

        void getConstraint2DerivativesState(constraint2_state_matrix_t& F) override
        {
          typename ComKinConstraintBase_t::constraint2_state_matrix_t F_comKinoDynamics;
          comKinConstraintBase_.getConstraint2DerivativesState(F_comKinoDynamics);
          F.template block<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE>(0, 0) = F_comKinoDynamics;
          F.template block<12+JOINT_COORD_SIZE, INPUTFILTER_INPUT_SIZE>(0, 12+JOINT_COORD_SIZE).setZero();
        }

        void getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F_final) override {
          typename ComKinConstraintBase_t::constraint2_state_matrix_t F_final_comKinoDynamics;
          comKinConstraintBase_.getConstraint2DerivativesState(F_final_comKinoDynamics);
          F_final.template block<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE>(0, 0) = F_final_comKinoDynamics;
          F_final.template block<12+JOINT_COORD_SIZE, INPUTFILTER_INPUT_SIZE>(0, 12+JOINT_COORD_SIZE).setZero();
        }

        void setStanceLegs (const contact_flag_t& stanceLegs) {
          comKinConstraintBase_.setStanceLegs(stanceLegs);
        }

        void getStanceLegs (contact_flag_t& stanceLegs) {
          comKinConstraintBase_.getStanceLegs(stanceLegs);
        };

    private:
        ComKinConstraintBase_t comKinConstraintBase_;
        input_filter_t inputFilter_;
    };
}


#endif //OCS2_COMKINOCONSTRAINTAUGMENTED_H
