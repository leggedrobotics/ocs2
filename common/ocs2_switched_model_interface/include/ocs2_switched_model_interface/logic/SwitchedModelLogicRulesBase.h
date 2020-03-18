/*
 * SwitchedModelLogicRulesBase.h
 *
 *  Created on: Nov 28, 2017
 *      Author: farbod
 */

#pragma once

#include <mutex>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/logic/ModeSchedule.h>
#include <ocs2_core/misc/Lookup.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/foot_planner/FeetPlannerBase.h"
#include "ocs2_switched_model_interface/foot_planner/SplineCpg.h"

namespace switched_model {

/**
 * Logic rules base class
 */
class SwitchedModelLogicRulesBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using scalar_t = ocs2::Dimensions<0, 0>::scalar_t;
  using scalar_array_t = ocs2::Dimensions<0, 0>::scalar_array_t;

  using foot_cpg_t = CPG_BASE<scalar_t>;
  using feet_planner_t = FeetPlannerBase<scalar_t, foot_cpg_t>;
  using feet_planner_ptr_t = std::shared_ptr<feet_planner_t>;
  using feet_cpg_ptr_t = feet_planner_t::feet_cpg_ptr_t;
  using feet_cpg_const_ptr_t = feet_planner_t::feet_cpg_const_ptr_t;
  using mode_sequence_template_t = ModeSequenceTemplate<scalar_t>;

  /**
   * Constructor
   *
   * @param [in] feetPlannerPtr: A pointer to the FeetPlanner class.
   * @param [in] phaseTransitionStanceTime: The phase transition stance time.
   */
  SwitchedModelLogicRulesBase(std::shared_ptr<feet_planner_t> feetPlannerPtr, scalar_t phaseTransitionStanceTime);

  /**
   * Copy constructor
   */
  SwitchedModelLogicRulesBase(const SwitchedModelLogicRulesBase& rhs);

  /**
   * Destructor
   */
  ~SwitchedModelLogicRulesBase() = default;

  /**
   * Retrieves constraints information of the motion phase specified by the given index. This information
   * is sufficient to determine the active constraints of the requested motion phase.
   *
   * @param [in] index: The requested motion phase.
   * @param [in] modeSchedule: The current ModeSchedule.
   * @param [out] contactFlags: A contact flags array which determines the stance legs.
   * @param [out] feetReferencePtr: An array of planned motion for the swing legs (note that
   * for the stance legs the behavior might not be defined).
   */
  void getMotionPhaseLogics(const size_t& index, contact_flag_t& contactFlags, std::array<const foot_cpg_t*, 4>& feetReferencePtr) const;

  /**
   * Gets a reference to the feet planner class.
   *
   * @return feet planner class.
   */
  const feet_planner_t& getFeetPlanner() const;

  /**
   * This method can be used to update the internal variables. This method will be called by any
   * program that tries to update the logic rules variables.
   */
  void update() override;

  /**
   * Rewinds the class. This method is only called in the MPC class.
   *
   * @param [in] lowerBoundTime: The smallest time for which the logicRules should be defined.
   * @param [in] upperBoundTime: The greatest time for which the logicRules should be defined.
   */
  void rewind(const scalar_t& lowerBoundTime, const scalar_t& upperBoundTime) override;

 protected:
  /**
   * Used in the SLQ-MPC method to insert a new user defined logic in the given time period.
   *
   * @param [in] modeSequenceTemplate: A data type which includes all necessary information for modifying the logicRules.
   * @param [in] startTime: The initial time from which the new logicRules template should be augmented.
   * @param [in] finalTime: The final time to which the new logicRules template should be augmented.
   */
  void insertModeSequenceTemplate(const logic_template_type& modeSequenceTemplate, const scalar_t& startTime,
                                  const scalar_t& finalTime) override;

  /**
   * This method can be used to reset the internal variables related to feet Z reference planner.
   */
  void resetFeetReferenceStock(size_t numModes);

 private:
  /**
   * Extends the switch information from lowerBoundTime to upperBoundTime based on the template mode sequence.
   *
   * @param modeSequenceTemplate: Mode sequence template which should be tiled into logic rule.
   * @param [in] startTime: The initial time from which the logicRules template should be augmented.
   * @param [in] finalTime: The final time to which the logicRules template should be augmented.
   * @param [in] modeSequenceTemplate: Mode sequence template which should be tiled into modeSchedule.
   * @param modeSchedule: The updated ModeSchedule.
   */
  void tileModeSequenceTemplate(const logic_template_type& modeSequenceTemplate, const scalar_t& startTime, const scalar_t& finalTime);

 private:
  std::mutex feetReferenceUpdateMutex_;
  std::vector<feet_cpg_ptr_t> feetReferencePtrStock_;
  std::vector<bool> feetReferenceUpdatedStock_;

  std::shared_ptr<feet_planner_t> feetPlannerPtr_;
  scalar_t phaseTransitionStanceTime_;
};

}  // namespace switched_model
