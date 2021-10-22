#pragma once

#include <raisim/object/terrain/HeightMap.hpp>

#include <ocs2_legged_robot/visualization/LeggedRobotVisualizer.h>

namespace ocs2 {
namespace legged_robot {

class LeggedRobotRaisimVisualizer : public LeggedRobotVisualizer {
 public:
  LeggedRobotRaisimVisualizer(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                              const PinocchioEndEffectorKinematics& endEffectorKinematics, ros::NodeHandle& nodeHandle,
                              scalar_t maxUpdateFrequency = 100.0, raisim::HeightMap const* terrain = nullptr);

  ~LeggedRobotRaisimVisualizer() override = default;

  void update(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command) override;

 private:
  raisim::HeightMap const* terrain_ = nullptr;
};

}  // namespace legged_robot
}  // namespace ocs2
