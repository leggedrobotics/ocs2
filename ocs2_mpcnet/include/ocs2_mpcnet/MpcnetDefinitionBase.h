#pragma once

#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>

namespace ocs2 {

/**
 * Base class for MPC-Net definitions.
 */
class MpcnetDefinitionBase {
 public:
  /**
   * Default destructor.
   */
  virtual ~MpcnetDefinitionBase() = default;

  /**
   * Deleted copy constructor.
   */
  MpcnetDefinitionBase(const MpcnetDefinitionBase&) = delete;

  /**
   * Deleted copy assignment.
   */
  MpcnetDefinitionBase& operator=(const MpcnetDefinitionBase&) = delete;

  /**
   * Get the generalized time.
   * @param[in] t : Absolute time.
   * @param[in] modeSchedule : Mode schedule.
   * @return The generalized time.
   */
  virtual vector_t getGeneralizedTime(scalar_t t, const ModeSchedule& modeSchedule) = 0;

  /**
   * Get the relative state.
   * @param[in] t : Absolute time.
   * @param[in] x : Robot state.
   * @param[in] targetTrajectories : Target trajectories.
   * @return The relative state.
   */
  virtual vector_t getRelativeState(scalar_t t, const vector_t& x, const TargetTrajectories& targetTrajectories) = 0;

  /**
   * Get the input transformation.
   * @param[in] t : Absolute time.
   * @param[in] x : Robot state.
   * @return The input transformation.
   */
  virtual matrix_t getInputTransformation(scalar_t t, const vector_t& x) = 0;

  /**
   * Check if a state is valid.
   * @param [in] x : State.
   * @return True if valid.
   */
  virtual bool validState(const vector_t& x) = 0;
};

}  // namespace ocs2
