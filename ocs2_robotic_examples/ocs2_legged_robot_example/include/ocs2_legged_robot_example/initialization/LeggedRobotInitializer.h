#pragma once

#include <ocs2_core/initialization/Initializer.h>

#include <ocs2_legged_robot_example/common/definitions.h>
#include <ocs2_legged_robot_example/logic/SwitchedModelModeScheduleManager.h>

namespace ocs2 {
namespace legged_robot {
class LeggedRobotInitializer : public ocs2::Initializer {
 public:
  using Base = ocs2::Initializer;

  LeggedRobotInitializer(std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr);

  LeggedRobotInitializer(const LeggedRobotInitializer& rhs);

  ~LeggedRobotInitializer() = default;

  LeggedRobotInitializer* clone() const override;

  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

 private:
  input_vector_t computeInputOperatingPoints(const contact_flag_t& contactFlags, const state_vector_t& nominalState) const;

  std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr_;
};
}  // namespace legged_robot
}  // namespace ocs2
