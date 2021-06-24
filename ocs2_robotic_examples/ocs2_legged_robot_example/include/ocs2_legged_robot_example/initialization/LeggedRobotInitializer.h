#pragma once

#include <ocs2_core/initialization/Initializer.h>

#include <ocs2_legged_robot_example/common/definitions.h>
#include <ocs2_legged_robot_example/logic/SwitchedModelModeScheduleManager.h>

namespace ocs2 {
namespace legged_robot {

class LeggedRobotInitializer final : public ocs2::Initializer {
 public:
  using Base = ocs2::Initializer;

  LeggedRobotInitializer(const SwitchedModelModeScheduleManager& modeScheduleManager);

  ~LeggedRobotInitializer() override = default;

  LeggedRobotInitializer* clone() const override;

  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

 private:
  LeggedRobotInitializer(const LeggedRobotInitializer& other) = default;

  const SwitchedModelModeScheduleManager* modeScheduleManagerPtr_;
};

}  // namespace legged_robot
}  // namespace ocs2
