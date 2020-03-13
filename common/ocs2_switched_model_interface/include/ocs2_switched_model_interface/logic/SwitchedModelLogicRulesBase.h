/*
 * SwitchedModelLogicRulesBase.h
 *
 *  Created on: Nov 28, 2017
 *      Author: farbod
 */

#pragma once

#include <mutex>

#include <ocs2_core/logic/rules/HybridLogicRules.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/foot_planner/FeetPlannerBase.h"
#include "ocs2_switched_model_interface/foot_planner/SplineCpg.h"

namespace switched_model {

/**
 * Logic rules base class
 */
class SwitchedModelLogicRulesBase : public ocs2::HybridLogicRules {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ocs2::HybridLogicRules;
  using typename BASE::logic_template_type;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::size_array_t;

  using foot_cpg_t = SplineCpg;
  using feet_planner_t = FeetPlannerBase;
  using feet_cpg_ptr_t = FeetPlannerBase::feet_cpg_ptr_t;

 public:
  /**
   * Default constructor
   */
  SwitchedModelLogicRulesBase() = default;

  /**
   * Constructor
   *
   * @param [in] feetPlannerPtr: A pointer to the FeetPlanner class.
   * @param [in] phaseTransitionStanceTime: The phase transition stance time.
   */
  explicit SwitchedModelLogicRulesBase(std::shared_ptr<feet_planner_t> feetPlannerPtr, scalar_t phaseTransitionStanceTime = 0.4);

  /**
   * Copy constructor
   */
  SwitchedModelLogicRulesBase(const SwitchedModelLogicRulesBase& rhs);

  /**
   * Destructor
   */
  ~SwitchedModelLogicRulesBase() override = default;

  /**
   * Move assignment
   */
  SwitchedModelLogicRulesBase& operator=(SwitchedModelLogicRulesBase&& other);

  /**
   * Assignment
   */
  SwitchedModelLogicRulesBase& operator=(const SwitchedModelLogicRulesBase& other);

  /**
   * Retrieves the contact status flag sequence.
   *
   * @return contactFlagsStock_
   */
  const std::vector<contact_flag_t>& getContactFlagsSequence() const;

  /**
   * Retrieves the contact flags of the requested motion phase.
   *
   * @param [in] index: The requested motion phase.
   * @param [out] contactFlags: A contact flags array which determines the stance legs.
   */
  void getContactFlags(const size_t& index, contact_flag_t& contactFlags) const;

  /**
   * Retrieves constraints information of the motion phase specified by the given index. This information
   * is sufficient to determine the active constraints of the requested motion phase.
   *
   * @param [in] index: The requested motion phase.
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
   * Extends the switch information from lowerBoundTime to upperBoundTime based on the template mode sequence.
   *
   * @param modeSequenceTemplate: Mode sequence template which should be tiled into logic rule.
   * @param [in] startTime: The initial time from which the logicRules template should be augmented.
   * @param [in] finalTime: The final time to which the logicRules template should be augmented.
   */
  void tileModeSequenceTemplate(const logic_template_type& modeSequenceTemplate, const scalar_t& startTime, const scalar_t& finalTime);

 private:
  mutable std::mutex feetReferenceUpdateMutex_;
  mutable std::vector<feet_cpg_ptr_t> feetReferencePtrStock_;
  mutable std::vector<bool> feetReferenceUpdatedStock_;

  std::shared_ptr<feet_planner_t> feetPlannerPtr_;

  scalar_t phaseTransitionStanceTime_;

  std::vector<contact_flag_t> contactFlagsStock_;
};

}  // namespace switched_model