#pragma once

#include <ocs2_core/logic/ModeSchedule.h>
#include <ocs2_core/misc/Lookup.h>
#include <mutex>

#include <ocs2_legged_robot_example/common/definitions.h>
#include <ocs2_legged_robot_example/logic/ModeSequenceTemplate.h>

namespace ocs2 {
namespace legged_robot {

class GaitSchedule {
 public:
  GaitSchedule(ocs2::ModeSchedule initModeSchedule, ModeSequenceTemplate initModeSequenceTemplate, scalar_t phaseTransitionStanceTime);

  /**
   * @param [in] lowerBoundTime: The smallest time for which the ModeSchedule should be defined.
   * @param [in] upperBoundTime: The greatest time for which the ModeSchedule should be defined.
   */
  ocs2::ModeSchedule getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime);

  /**
   * Used to insert a new user defined logic in the given time period.
   *
   * @param [in] startTime: The initial time from which the new mode sequence template should start.
   * @param [in] finalTime: The final time until when the new mode sequence needs to be defined.
   */
  void insertModeSequenceTemplate(const ModeSequenceTemplate& modeSequenceTemplate, scalar_t startTime, scalar_t finalTime);

 private:
  /**
   * Extends the switch information from lowerBoundTime to upperBoundTime based on the template mode sequence.
   *
   * @param [in] startTime: The initial time from which the mode schedule should be appended with the template.
   * @param [in] finalTime: The final time to which the mode schedule should be appended with the template.
   */
  void tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime);

 private:
  ocs2::ModeSchedule modeSchedule_;
  ModeSequenceTemplate modeSequenceTemplate_;
  scalar_t phaseTransitionStanceTime_;
};

}  // namespace legged_robot
}  // namespace ocs2
