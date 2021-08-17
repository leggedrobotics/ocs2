#pragma once

#include <ocs2_mpcnet/MpcnetDefinitionBase.h>

namespace ocs2 {
namespace ballbot {

/**
 * MPC-Net definitions for ballbot.
 */
class BallbotMpcnetDefinition : public MpcnetDefinitionBase {
 public:
  /**
   * Default constructor.
   */
  BallbotMpcnetDefinition() = default;

  /**
   * Default destructor.
   */
  ~BallbotMpcnetDefinition() override = default;

  /**
   * @see MpcnetDefinitionBase::getGeneralizedTime
   */
  vector_t getGeneralizedTime(scalar_t t, const ModeSchedule& modeSchedule) override;

  /**
   * @see MpcnetDefinitionBase::getRelativeState
   */
  vector_t getRelativeState(scalar_t t, const vector_t& x, const TargetTrajectories& targetTrajectories) override;

  /**
   * @see MpcnetDefinitionBase::validState
   */
  bool validState(const vector_t& x) override;
};

}  // namespace ballbot
}  // namespace ocs2
