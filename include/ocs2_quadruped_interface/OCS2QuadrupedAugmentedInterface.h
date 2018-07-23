//
// Created by ruben on 23.07.18.
//

#ifndef OCS2_OCS_QUADRUPEDAUGMENTEDINTERFACE_H
#define OCS2_OCS_QUADRUPEDAUGMENTEDINTERFACE_H

#include "OCS2QuadrupedInterface.h"

template<size_t JOINT_COORD_SIZE, size_t INPUTFILTER_STATE_SIZE, size_t INPUTFILTER_INPUT_SIZE>
class OCS2QuadrupedAugmentedInterface : public OCS2QuadrupedInterface<JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
    12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BASE = OCS2QuadrupedInterface<JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE+INPUTFILTER_STATE_SIZE,
        12+JOINT_COORD_SIZE+INPUTFILTER_INPUT_SIZE>;
    using kinematic_model_t = typename BASE::kinematic_model_t;
    using com_model_t = typename BASE::com_model_t;
    using state_estimator_t = typename BASE::state_estimator_t;
    using rbd_state_vector_t = typename BASE::rbd_state_vector_t;
    using state_vector_t = typename BASE::state_vector_t;
    using input_vector_t = typename BASE::input_vector_t;

    using filter_state_t = Eigen::Matrix<double, INPUTFILTER_STATE_SIZE, 1>;

    OCS2QuadrupedAugmentedInterface(
        const kinematic_model_t& kinematicModel,
        const com_model_t& comModel,
        const std::string& pathToConfigFolder) :
    BASE(kinematicModel, comModel, pathToConfigFolder) {};

    virtual ~OCS2QuadrupedAugmentedInterface() = default;

    void computeSwitchedModelState(
        const rbd_state_vector_t& rbdState,
        state_vector_t& extended_state) override
    {
        state_estimator_t::comkino_model_state_t comkinoState;
        switchedModelStateEstimator_.estimateComkinoModelState(rbdState, comkinoState);
        extended_state.template segment<12+JOINT_COORD_SIZE>(0) = comkinoState;
        extended_state.template segment<INPUTFILTER_STATE_SIZE>(12+JOINT_COORD_SIZE).setZero();
      };

    void computeSwitchedModelState(
        const rbd_state_vector_t& rbdState,
        const filter_state_t& filterState,
        state_vector_t& extended_state)
    {
      state_estimator_t::comkino_model_state_t comkinoState;
      switchedModelStateEstimator_.estimateComkinoModelState(rbdState, comkinoState);
      extended_state.template segment<12+JOINT_COORD_SIZE>(0) = comkinoState;
      extended_state.template segment<INPUTFILTER_STATE_SIZE>(12+JOINT_COORD_SIZE) = filterState;
    };

    void computeRbdModelState(
        const state_vector_t& extended_state,
        const input_vector_t& extended_input,
        rbd_state_vector_t& rbdState) override
    {
      state_estimator_t::comkino_model_state_t comkinoState = extended_input.template segment<12+JOINT_COORD_SIZE>(0);
      state_estimator_t::joint_coordinate_t dqJoints = extended_input.template segment<JOINT_COORD_SIZE>(JOINT_COORD_SIZE);
      switchedModelStateEstimator_.estimateRbdModelState(comkinoState, dqJoints, rbdState);
    };
};


#endif //OCS2_OCS_QUADRUPEDAUGMENTEDINTERFACE_H
