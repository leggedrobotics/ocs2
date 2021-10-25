#pragma once

#include <ocs2_mpcnet/MpcnetDefinitionBase.h>

namespace ocs2 {
namespace legged_robot {

/**
 * MPC-Net definitions for legged robot.
 */
class LeggedRobotMpcnetDefinition : public MpcnetDefinitionBase {
 public:
  /**
   * Constructor.
   * @param [in] defaultState : Default state.
   */
  LeggedRobotMpcnetDefinition(const vector_t& defaultState) : defaultState_(defaultState) {}

  /**
   * Default destructor.
   */
  ~LeggedRobotMpcnetDefinition() override = default;

  /**
   * @see MpcnetDefinitionBase::getGeneralizedTime
   */
  vector_t getGeneralizedTime(scalar_t t, const ModeSchedule& modeSchedule) override;

  /**
   * @see MpcnetDefinitionBase::getRelativeState
   */
  vector_t getRelativeState(scalar_t t, const vector_t& x, const TargetTrajectories& targetTrajectories) override;

  /**
   * @see MpcnetDefinitionBase::getInputTransformation
   */
  matrix_t getInputTransformation(scalar_t t, const vector_t& x) override;

  /**
   * @see MpcnetDefinitionBase::validState
   */
  bool validState(const vector_t& x) override;

 private:
  vector_t defaultState_;
};

}  // namespace legged_robot
}  // namespace ocs2
