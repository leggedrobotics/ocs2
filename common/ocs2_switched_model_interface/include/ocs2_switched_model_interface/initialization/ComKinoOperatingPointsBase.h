#pragma once

#include <ocs2_core/initialization/SystemOperatingPoint.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h"

namespace switched_model {

class ComKinoOperatingPointsBase : public ocs2::SystemOperatingPoint<24, 24> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr size_t JOINT_COORD_SIZE = 12;
  static constexpr size_t STATE_DIM = 24;
  static constexpr size_t INPUT_DIM = 24;
  static constexpr size_t NUM_CONTACT_POINTS_ = SwitchedModel<JOINT_COORD_SIZE>::NUM_CONTACT_POINTS;

  using Base = ocs2::SystemOperatingPoint<STATE_DIM, INPUT_DIM>;
  using typename Base::input_vector_array_t;
  using typename Base::input_vector_t;
  using typename Base::scalar_array_t;
  using typename Base::scalar_t;
  using typename Base::state_vector_array_t;
  using typename Base::state_vector_t;

  using com_model_t = ComModelBase<JOINT_COORD_SIZE>;
  using logic_rules_t = SwitchedModelPlannerLogicRules<JOINT_COORD_SIZE, double>;
  using contact_flag_t = typename SwitchedModel<JOINT_COORD_SIZE>::contact_flag_t;

  ComKinoOperatingPointsBase(const com_model_t& comModel, std::shared_ptr<const logic_rules_t> logicRulesPtr);

  ComKinoOperatingPointsBase(const ComKinoOperatingPointsBase& rhs);

  ~ComKinoOperatingPointsBase() override = default;

  ComKinoOperatingPointsBase* clone() const override;

  void getSystemOperatingTrajectories(const state_vector_t& initialState, const scalar_t& startTime, const scalar_t& finalTime,
                                              scalar_array_t& timeTrajectory, state_vector_array_t& stateTrajectory,
                                              input_vector_array_t& inputTrajectory, bool concatOutput = false) override;

 private:

  void computeInputOperatingPoints(contact_flag_t contactFlags, input_vector_t& inputs);

  std::unique_ptr<com_model_t> comModelPtr_;
  std::shared_ptr<const logic_rules_t> logicRulesPtr_;
};

}  // end of namespace switched_model

