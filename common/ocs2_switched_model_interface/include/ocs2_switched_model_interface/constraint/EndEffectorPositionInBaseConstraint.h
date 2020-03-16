#pragma once

#include <ocs2_switched_model_interface/constraint/EndEffectorConstraint.h>
#include <ocs2_switched_model_interface/constraint/EndEffectorPositionConstraint.h>

namespace switched_model {

class EndEffectorPositionInBaseConstraint final : public EndEffectorConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = switched_model::EndEffectorConstraint;
  using typename BASE::ad_com_model_t;
  using typename BASE::ad_dynamic_vector_t;
  using typename BASE::ad_interface_t;
  using typename BASE::ad_kinematic_model_t;
  using typename BASE::ad_scalar_t;
  using typename BASE::constraint_timeStateInput_matrix_t;
  using typename BASE::dynamic_vector_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_t;
  using typename BASE::LinearApproximation_t;
  using typename BASE::QuadraticApproximation_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;
  using typename BASE::timeStateInput_matrix_t;

  explicit EndEffectorPositionInBaseConstraint(int legNumber, EndEffectorPositionConstraintSettings settings, ad_com_model_t& adComModel,
                                               ad_kinematic_model_t& adKinematicsModel, bool generateModels = true,
                                               std::string constraintPrefix = "b_EEPositionConstraint_")
      : BASE(ocs2::ConstraintOrder::Linear, std::move(constraintPrefix), legNumber, std::move(settings), adComModel, adKinematicsModel,
             EndEffectorPositionInBaseConstraint::adfunc, generateModels) {}

  EndEffectorPositionInBaseConstraint(const EndEffectorPositionInBaseConstraint& rhs) = default;

  EndEffectorPositionInBaseConstraint* clone() const override { return new EndEffectorPositionInBaseConstraint(*this); }

 private:
  static void adfunc(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, int legNumber,
                     const ad_dynamic_vector_t& tapedInput, ad_dynamic_vector_t& o_footPosition) {
    // Extract elements from taped input
    ad_scalar_t t = tapedInput(0);
    comkino_state_ad_t x = tapedInput.segment(1, STATE_DIM);
    comkino_input_ad_t u = tapedInput.segment(1 + STATE_DIM, INPUT_DIM);

    // Extract elements from state
    const base_coordinate_ad_t comPose = getComPose(x);
    const joint_coordinate_ad_t qJoints = getJointPositions(x);

    // Get base state from com state
    const base_coordinate_ad_t basePose = adComModel.calculateBasePose(comPose);

    o_footPosition = adKinematicsModel.positionBaseToFootInBaseFrame(legNumber, jointPositions);
  }

};
}  // namespace switched_model
