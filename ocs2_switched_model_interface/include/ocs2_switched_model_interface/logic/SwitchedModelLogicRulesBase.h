/*
 * SwitchedModelLogicRulesBase.h
 *
 *  Created on: Nov 28, 2017
 *      Author: farbod
 */

#ifndef SWITCHEDMODELLOGICRULESBASE_H_
#define SWITCHEDMODELLOGICRULESBASE_H_

#include <mutex>

#include <ocs2_core/logic/rules/HybridLogicRules.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/foot_planner/FeetPlannerBase.h"
#include "ocs2_switched_model_interface/foot_planner/cpg/CPG_BASE.h"
#include "ocs2_switched_model_interface/state_constraint/EndEffectorConstraintBase.h"

namespace switched_model {

/**
 * Logic rules base class
 */
template <size_t JOINT_COORD_SIZE, class cpg_t>
class SwitchedModelLogicRulesBase : public ocs2::HybridLogicRules {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<SwitchedModelLogicRulesBase<JOINT_COORD_SIZE, cpg_t>> Ptr;

  typedef ocs2::HybridLogicRules BASE;

  typedef typename BASE::size_array_t size_array_t;
  typedef typename BASE::scalar_t scalar_t;
  typedef typename BASE::scalar_array_t scalar_array_t;

  typedef typename BASE::logic_template_type logic_template_type;

  typedef SwitchedModel<JOINT_COORD_SIZE> switched_model_t;
  typedef typename switched_model_t::contact_flag_t contact_flag_t;

  typedef FeetPlannerBase<scalar_t, cpg_t> feet_planner_t;
  typedef typename feet_planner_t::Ptr feet_planner_ptr_t;
  typedef cpg_t foot_cpg_t;
  typedef typename feet_planner_t::feet_cpg_ptr_t feet_cpg_ptr_t;
  typedef typename feet_planner_t::feet_cpg_const_ptr_t feet_cpg_const_ptr_t;

 private:
  using read_lock_t = std::unique_lock<std::mutex>;
  using write_lock_t = std::unique_lock<std::mutex>;

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
  SwitchedModelLogicRulesBase(const feet_planner_ptr_t& feetPlannerPtr, const scalar_t& phaseTransitionStanceTime = 0.4);

  /**
   * Copy constructor
   */
  SwitchedModelLogicRulesBase(const SwitchedModelLogicRulesBase& rhs);

  /**
   * Destructor
   */
  virtual ~SwitchedModelLogicRulesBase() = default;

  /**
   * Move assignment
   */
  SwitchedModelLogicRulesBase& operator=(SwitchedModelLogicRulesBase&& other);

  /**
   * Assignment
   */
  SwitchedModelLogicRulesBase& operator=(const SwitchedModelLogicRulesBase& other);

  /**
   * Sets motion constraints which include mode sequence constraints and state-only constraint defined by gaps, etc.
   *
   * @param [in] subsystemsSequence: The sequence of the triggered subsystems.
   * @param [in] eventTimes: The sequence of the times in which mode transition took place.
   * @param [in] endEffectorStateConstraints: An array of the state-only constraint defined by gaps, etc.
   */
  void setMotionConstraints(const size_array_t& subsystemsSequence, const scalar_array_t& eventTimes,
                            const std::vector<EndEffectorConstraintBase::ConstPtr>& endEffectorStateConstraints =
                                std::vector<EndEffectorConstraintBase::ConstPtr>());

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
  void getMotionPhaseLogics(const size_t& index, contact_flag_t& contactFlags, std::array<const cpg_t*, 4>& feetReferencePtr) const;

  /**
   * Get a constant pointer to state-only constraints for end-effectors.
   *
   * @return A constant pointer to the array of constant pointer for endEffectorStateConstraint objects.
   */
  const std::vector<EndEffectorConstraintBase::ConstPtr>* getEndEffectorStateConstraintsPtr() const;

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
  virtual void update() override;

  /**
   * Rewinds the class. This method is only called in the MPC class.
   *
   * @param [in] lowerBoundTime: The smallest time for which the logicRules should be defined.
   * @param [in] upperBoundTime: The greatest time for which the logicRules should be defined.
   */
  virtual void rewind(const scalar_t& lowerBoundTime, const scalar_t& upperBoundTime) override;

 protected:
  /**
   * Used in the SLQ-MPC method to insert a new user defined logic in the given time period.
   *
   * @param [in] modeSequenceTemplate: A data type which includes all necessary information for modifying the logicRules.
   * @param [in] startTime: The initial time from which the new logicRules template should be augmented.
   * @param [in] finalTime: The final time to which the new logicRules template should be augmented.
   */
  virtual void insertModeSequenceTemplate(const logic_template_type& modeSequenceTemplate, const scalar_t& startTime,
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

  feet_planner_ptr_t feetPlannerPtr_;

  scalar_t phaseTransitionStanceTime_;

  std::vector<contact_flag_t> contactFlagsStock_;


  std::vector<EndEffectorConstraintBase::ConstPtr> endEffectorStateConstraints_;
};

/**
 * Specialization for planner which uses only Z direction planner
 */
template <size_t JOINT_COORD_SIZE, typename scalar_t = double>
using SwitchedModelPlannerLogicRules = SwitchedModelLogicRulesBase<JOINT_COORD_SIZE, CPG_BASE<scalar_t>>;

}  // namespace switched_model

#include "implementation/SwitchedModelLogicRulesBase.h"

#endif /* SWITCHEDMODELLOGICRULESBASE_H_ */
