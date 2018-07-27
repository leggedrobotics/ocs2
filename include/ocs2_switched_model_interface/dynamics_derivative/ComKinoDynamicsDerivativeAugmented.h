//
// Created by ruben on 12.07.18.
//

#ifndef OCS2_COMKINODYNAMICSDERIVATIVEAUGMENTED_H
#define OCS2_COMKINODYNAMICSDERIVATIVEAUGMENTED_H

#include "ComKinoDynamicsDerivativeBase.h"
#include "ocs2_switched_model_interface/dynamics/filterDynamics.h"

namespace switched_model {

    template<size_t JOINT_COORD_SIZE, size_t INPUTFILTER_STATE_SIZE, size_t INPUTFILTER_INPUT_SIZE>
    class ComKinoDynamicsDerivativeAugmented : public ocs2::DerivativesBase<
        12 + JOINT_COORD_SIZE + INPUTFILTER_STATE_SIZE,
        12 + JOINT_COORD_SIZE + INPUTFILTER_INPUT_SIZE,
        SwitchedModelPlannerLogicRules<
        JOINT_COORD_SIZE,
        12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
        12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE>>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using ComKinoDynamicsDerivativeBase_t = ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE,
            12+JOINT_COORD_SIZE,
            12+JOINT_COORD_SIZE,
            SwitchedModelPlannerLogicRules<
            JOINT_COORD_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE>>;
        using kinematic_model_t = typename ComKinoDynamicsDerivativeBase_t::kinematic_model_t;
        using com_model_t = typename ComKinoDynamicsDerivativeBase_t::com_model_t;
        using contact_flag_t = typename ComKinoDynamicsDerivativeBase_t::contact_flag_t;

        using logic_rules_t = SwitchedModelPlannerLogicRules<
            JOINT_COORD_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE>;
        using logic_rules_machine_t = typename ocs2::LogicRulesMachine<12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE, logic_rules_t>;

        using input_filter_t = FilterDynamics<INPUTFILTER_STATE_SIZE, INPUTFILTER_INPUT_SIZE, 12+JOINT_COORD_SIZE>;

        using BASE = ocs2::DerivativesBase<
            12 + JOINT_COORD_SIZE + INPUTFILTER_STATE_SIZE,
            12 + JOINT_COORD_SIZE + INPUTFILTER_INPUT_SIZE,
            SwitchedModelPlannerLogicRules<
            JOINT_COORD_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
            12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE>>;
        using scalar_t = typename BASE::scalar_t;
        using state_vector_t = typename BASE::state_vector_t;
        using input_vector_t = typename BASE::input_vector_t;
        using state_matrix_t = typename BASE::state_matrix_t;
        using state_input_matrix_t = typename BASE::state_input_matrix_t;

        ComKinoDynamicsDerivativeAugmented(const FilterSettings& filterSettings,
                                           const kinematic_model_t& kinematicModel,
                                           const com_model_t& comModel,
                                           const Model_Settings& options = Model_Settings()) :
        comKinoDynamicsDerivativeBase_(kinematicModel, comModel, options),
        inputFilter_(filterSettings) {};

        ~ComKinoDynamicsDerivativeAugmented() override = default;

        ComKinoDynamicsDerivativeAugmented<JOINT_COORD_SIZE, INPUTFILTER_STATE_SIZE, INPUTFILTER_INPUT_SIZE>* clone() const override
        {
          return new ComKinoDynamicsDerivativeAugmented<JOINT_COORD_SIZE, INPUTFILTER_STATE_SIZE, INPUTFILTER_INPUT_SIZE>(*this);
        };

        void initializeModel(logic_rules_machine_t& logicRulesMachine,
                             const size_t& partitionIndex, const char* algorithmName=NULL) override
        {
          comKinoDynamicsDerivativeBase_.initializeModel(logicRulesMachine, partitionIndex, algorithmName);
        };

        void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override
        {
          BASE::setCurrentStateAndControl(t, x, u);
#ifndef NDEBUG
          std::cout << "==== x: t=" << BASE::t_ << "====" << std::endl;
          std::cout << BASE::x_.transpose() << std::endl;
          std::cout << "==== u: t=" << BASE::t_ << "====" << std::endl;
          std::cout << BASE::u_.transpose() << std::endl;
#endif
          typename ComKinoDynamicsDerivativeBase_t::state_vector_t x_comKinoDynamics = x.template segment<12+JOINT_COORD_SIZE>(0);
          typename ComKinoDynamicsDerivativeBase_t::input_vector_t u_comKinDynamics = u.template segment<12+JOINT_COORD_SIZE>(0);
          comKinoDynamicsDerivativeBase_.setCurrentStateAndControl(t, x_comKinoDynamics, u_comKinDynamics);
        };

        void getFlowMapDerivativeState(state_matrix_t& A) override
        {
          // comKinoDynamics block
          typename ComKinoDynamicsDerivativeBase_t::state_matrix_t A_comKinoDynamics;
          comKinoDynamicsDerivativeBase_.getFlowMapDerivativeState(A_comKinoDynamics);
          A.template block<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE>(0, 0) = A_comKinoDynamics;

          // input filter block
          A.template block<INPUTFILTER_STATE_SIZE, INPUTFILTER_STATE_SIZE>(12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE) =
              inputFilter_.getA();

          // Off diagonal coupling blocks
          A.template block<12+JOINT_COORD_SIZE, INPUTFILTER_STATE_SIZE>(0, 12+JOINT_COORD_SIZE).setZero();
          A.template block<INPUTFILTER_STATE_SIZE, 12+JOINT_COORD_SIZE>(12+JOINT_COORD_SIZE, 0).setZero();
#ifndef NDEBUG
          std::cout << "==== A: t=" << BASE::t_ << "====" << std::endl;
          std::cout << A << std::endl;
#endif
        };

        void getFlowMapDerivativeInput(state_input_matrix_t& B)  override
        {
          // comKinoDynamics block
          typename ComKinoDynamicsDerivativeBase_t::state_input_matrix_t B_comKinoDynamics;
          comKinoDynamicsDerivativeBase_.getFlowMapDerivativeInput(B_comKinoDynamics);
          B.template block<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE>(0, 0) = B_comKinoDynamics;

          // input filter block
          B.template block<INPUTFILTER_STATE_SIZE, INPUTFILTER_INPUT_SIZE>(12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE) =
              inputFilter_.getB();

          // Off diagonal coupling blocks
          B.template block<12+JOINT_COORD_SIZE, INPUTFILTER_INPUT_SIZE>(0, 12+JOINT_COORD_SIZE).setZero();
          B.template block<INPUTFILTER_STATE_SIZE, 12+JOINT_COORD_SIZE>(12+JOINT_COORD_SIZE, 0).setZero();

#ifndef NDEBUG
          std::cout << "==== B: t=" << BASE::t_ << "====" << std::endl;
          std::cout << B << std::endl;
#endif
        }

        void setStanceLegs (const contact_flag_t& stanceLegs) {
          comKinoDynamicsDerivativeBase_.setStanceLegs(stanceLegs);
        }

        void getStanceLegs (contact_flag_t& stanceLegs) {
          comKinoDynamicsDerivativeBase_.getStanceLegs(stanceLegs);
        };

    private:
        ComKinoDynamicsDerivativeBase_t comKinoDynamicsDerivativeBase_;
        input_filter_t inputFilter_;

};
}; // namespace switched_model


#endif //OCS2_COMKINODYNAMICSDERIVATIVEAUGMENTED_H
