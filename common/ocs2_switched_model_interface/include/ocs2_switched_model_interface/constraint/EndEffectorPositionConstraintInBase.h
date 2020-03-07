#pragma once

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_switched_model_interface/constraint/ConstraintTerm.h>
#include <ocs2_switched_model_interface/constraint/EndEffectorConstraint.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"

#include <ocs2_switched_model_interface/constraint/EndEffectorPositionConstraint.h>
#include <ocs2_switched_model_interface/core/Rotations.h>

namespace switched_model {
namespace constraints {

class EndEffectorPositionConstraintInBase : public EndEffectorPositionConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = switched_model::EndEffectorPositionConstraint;
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

  explicit EndEffectorPositionConstraintInBase(int legNumber, EndEffectorPositionConstraintSettings settings, ad_com_model_t& adComModel,
                                               ad_kinematic_model_t& adKinematicsModel, bool generateModels,
                                               std::string constraintPrefix = "b_EEPositionConstraint_")
      : BASE(legNumber, std::move(settings), adComModel, adKinematicsModel, generateModels, constraintPrefix) {}

  EndEffectorPositionConstraintInBase(const EndEffectorPositionConstraintInBase& rhs) : BASE(rhs) {}

  EndEffectorPositionConstraintInBase* clone() const override { return new EndEffectorPositionConstraintInBase(*this); }

 private:
  void adFootPosition(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, const ad_dynamic_vector_t& tapedInput,
                      ad_dynamic_vector_t& o_footPosition) {
    // Extract elements from taped input
    ad_scalar_t t = tapedInput(0);
    comkino_state_ad_t x = tapedInput.segment(1, STATE_DIM);
    comkino_input_ad_t u = tapedInput.segment(1 + STATE_DIM, INPUT_DIM);

    // Extract elements from state
    const base_coordinate_ad_t comPose = getComPose(x);
    const joint_coordinate_ad_t qJoints = getJointPositions(x);

    // Get base state from com state
    const base_coordinate_ad_t basePose = adComModel.calculateBasePose(comPose);

    o_footPosition = adKinematicsModel.positionBaseToFootInBaseFrame(footIndex, jointPositions);

    void setAdInterface(ad_com_model_t & adComModel, ad_kinematic_model_t & adKinematicsModel) override {
      // Function to differentiate
      auto adfunc = [this, &adComModel, &adKinematicsModel](const ad_dynamic_vector_t& x, ad_dynamic_vector_t& y) {
        this->adFootPosition(adComModel, adKinematicsModel, x, y);
      };

      adInterface_.reset(new ad_interface_t(adfunc, BASE::range_dim_, BASE::domain_dim_, libName_, libFolder_));
    };
  };
}  // namespace constraints
}  // namespace constraints
