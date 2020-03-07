#pragma once

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_switched_model_interface/constraint/ConstraintTerm.h>
#include <ocs2_switched_model_interface/constraint/EndEffectorConstraint.h>

#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include <ocs2_switched_model_interface/core/Rotations.h>

namespace switched_model {

struct EndEffectorVelocityConstraintSettings : public EndEffectorConstraintSettings {
  EndEffectorVelocityConstraintSettings() = default;
  EndEffectorVelocityConstraintSettings(size_t rows, size_t cols) : EndEffectorConstraintSettings(rows, cols){};
};

class EndEffectorVelocityConstraint : public EndEffectorConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = EndEffectorConstraint;
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

  static constexpr ocs2::ConstraintOrder kConstraintOrder = ocs2::ConstraintOrder::Linear;

  explicit EndEffectorVelocityConstraint(int legNumber, EndEffectorVelocityConstraintSettings settings, ad_com_model_t& adComModel,
                                         ad_kinematic_model_t& adKinematicsModel, bool generateModels,
                                         std::string constraintPrefix = "EEVelocityConstraint_")
      : BASE(kConstraintOrder, constraintPrefix, legNumber, std::move(settings)) {
    initializeADInterface(adComModel, adKinematicsModel, generateModels);
  }

  explicit EndEffectorVelocityConstraint(int legNumber, EndEffectorVelocityConstraintSettings settings,
                                         std::string constraintPrefix = "EEVelocityConstraint_")
      : BASE(kConstraintOrder, constraintPrefix, legNumber, std::move(settings)) {}

  EndEffectorVelocityConstraint(const EndEffectorVelocityConstraint& rhs) : BASE(rhs) {}

  EndEffectorVelocityConstraint* clone() const override { return new EndEffectorVelocityConstraint(*this); }

  scalar_array_t getValue(scalar_t time, const state_vector_t& state, const input_vector_t& input) const override {
    // Assemble input
    dynamic_vector_t tapedInput(1 + STATE_DIM + INPUT_DIM);
    tapedInput << time, state, input;

    // Compute constraints
    dynamic_vector_t eeVelocityWorld = adInterface_->getFunctionValue(tapedInput);

    // Change to std::vector
    scalar_array_t constraintValue;
    Eigen::VectorXd values = settings_.A() * eeVelocityWorld + settings_.b();
    for (int i = 0; i < settings_.b().rows(); i++) {
      constraintValue.emplace_back(values[i]);
    }
    return constraintValue;
  };

 private:
  void adFootVelocity(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, const ad_dynamic_vector_t& tapedInput,
                      ad_dynamic_vector_t& o_footVelocity) {
    // Extract elements from taped input
    ad_scalar_t t = tapedInput(0);
    comkino_state_ad_t x = tapedInput.segment(1, STATE_DIM);
    comkino_input_ad_t u = tapedInput.segment(1 + STATE_DIM, INPUT_DIM);

    // Extract elements from state
    const base_coordinate_ad_t comPose = getComPose(x);
    const base_coordinate_ad_t com_comTwist = getComLocalVelocities(x);
    const joint_coordinate_ad_t qJoints = getJointPositions(x);
    const joint_coordinate_ad_t dqJoints = getJointVelocities(u);

    // Get base state from com state
    const base_coordinate_ad_t basePose = adComModel.calculateBasePose(comPose);
    const base_coordinate_ad_t com_baseTwist = adComModel.calculateBaseLocalVelocities(com_comTwist);

    o_footVelocity = adKinematicsModel.footVelocityInOriginFrame(legNumber_, basePose, com_baseTwist, qJoints, dqJoints);
  }

  void setAdInterface(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel) override {
    // Function to differentiate
    auto adfunc = [this, &adComModel, &adKinematicsModel](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) {
      this->adFootVelocity(adComModel, adKinematicsModel, x, y);
    };

    adInterface_.reset(new ad_interface_t(adfunc, BASE::range_dim_, BASE::domain_dim_, libName_, libFolder_));
  };
};

}  // namespace switched_model
