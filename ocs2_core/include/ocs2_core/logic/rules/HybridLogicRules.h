/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef HYBRIDLOGICRULES_OCS2_H_
#define HYBRIDLOGICRULES_OCS2_H_

#include <iostream>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/misc/Lookup.h>

namespace ocs2 {

/**
 * Mode sequence template.
 *
 */
template <typename scalar_t = double>
struct ModeSequenceTemplate {
  ModeSequenceTemplate() : templateSwitchingTimes_(0), templateSubsystemsSequence_(0) {}

  /**
   * Defined as [t_0=0, t_1, .., t_n, t_(n+1)=T], where T is the overall duration
   * of the template logic. t_1 to t_n are the event moments.
   */
  std::vector<scalar_t> templateSwitchingTimes_;

  /**
   * Defined as [sys_0, sys_n], are the switching systems IDs. Here sys_i is
   * active in period [t_i, t_(i+1)]
   */
  std::vector<size_t> templateSubsystemsSequence_;

  /**
   * Displays template information.
   */
  void display() const {
    std::cerr << std::endl << "Template switching times:\n\t {";
    for (auto& s : templateSwitchingTimes_) {
      std::cerr << s << ", ";
    }
    if (!templateSwitchingTimes_.empty()) {
      std::cerr << "\b\b";
    }
    std::cerr << "}" << std::endl;

    std::cerr << "Template subsystem sequence:\n\t {";
    for (auto& s : templateSubsystemsSequence_) {
      std::cerr << s << ", ";
    }
    if (!templateSubsystemsSequence_.empty()) {
      std::cerr << "\b\b";
    }
    std::cerr << "}" << std::endl;
  }
};

/**
 * Hybrid logic rules base class
 *
 */
class HybridLogicRules {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using DIMENSIONS = Dimensions<0, 0>;
  using size_array_t = typename DIMENSIONS::size_array_t;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;

  using logic_template_type = ModeSequenceTemplate<scalar_t>;

  /**
   * Default constructor
   */
  HybridLogicRules() : subsystemsSequence_(1, 0), eventTimes_(0) {}

  /**
   * Constructor
   */
  HybridLogicRules(scalar_array_t eventTimes, size_array_t subsystemsSequence)
      : subsystemsSequence_(std::move(subsystemsSequence)), eventTimes_(std::move(eventTimes)) {}

  /**
   * Copy constructor
   */
  HybridLogicRules(const HybridLogicRules& rhs) : eventTimes_(rhs.eventTimes_), subsystemsSequence_(rhs.subsystemsSequence_) {}

  /**
   * Destructor
   */
  virtual ~HybridLogicRules() = default;

  /**
   * Move assignment
   */
  HybridLogicRules& operator=(HybridLogicRules&& other) = default;

  /**
   * Assignment
   */
  HybridLogicRules& operator=(const HybridLogicRules& other) = default;

  /**
   * Displays event information.
   */
  void display() const {
    // Event Times
    std::cerr << std::endl << "Event Times:\n\t {";
    for (auto& t : eventTimes_) {
      std::cerr << t << ", ";
    }
    if (!eventTimes_.empty()) {
      std::cerr << "\b\b";
    }
    std::cerr << "}" << std::endl;

    // Event Count
    std::cerr << "Event counters:\n\t {";
    for (size_t i = 0; i <= eventTimes_.size(); i++) {
      std::cerr << i << ", ";
    }
    std::cerr << "\b\b}" << std::endl;

    // Subsystem sequence
    std::cerr << "Subsystem sequence:\n\t {";
    for (auto& s : subsystemsSequence_) {
      std::cerr << s << ", ";
    }
    if (!subsystemsSequence_.empty()) {
      std::cerr << "\b\b";
    }
    std::cerr << "}" << std::endl;
  }

  /**
   * Sets the mode sequence which include the sequence of the triggered subsystems and their respective
   * transition time sequence.
   * Note: The update method is called to update the derived class variables.
   *
   * @param [in] subsystemsSequence: The sequence of the triggered subsystems.
   * @param [in] eventTimes: The sequence of the times in which mode transition took place.
   */
  void setModeSequence(size_array_t subsystemsSequence, scalar_array_t eventTimes) {
    if (subsystemsSequence.size() != eventTimes.size() + 1) {
      throw std::runtime_error("The number of subsystems should be 1 plus the number of the event times.");
    }

    subsystemsSequence_ = std::move(subsystemsSequence);
    eventTimes_ = std::move(eventTimes);

    // update derived class internal variables
    update();
  }

  /**
   * Retrieves the sequence of the triggered subsystems.
   *
   * @return A constant reference to subsystemsSequence_.
   */
  const size_array_t& subsystemsSequence() const { return subsystemsSequence_; }

  /**
   * Retrieves the sequence of the triggered subsystems.
   *
   * @return A reference to subsystemsSequence_.
   */
  size_array_t& subsystemsSequence() { return subsystemsSequence_; }

  /**
   * Retrieves the number of the subsystems.
   *
   * @return number of subsystems.
   */
  size_t getNumSubsystems() const { return subsystemsSequence_.size(); }

  /**
   * Set the model sequence template.
   *
   * @param [in] modeSequenceTemplate: A data type which includes all necessary information for modifying the logicRules.
   */
  void setModeSequenceTemplate(const logic_template_type& modeSequenceTemplate) { modeSequenceTemplate_ = modeSequenceTemplate; }

  /**
   * Returns the model model sequence template.
   *
   * @return Logic Rules template.
   */
  const logic_template_type& modeSequenceTemplate() const { return modeSequenceTemplate_; }

  /**
   * Retrieves the event times.
   *
   * @return A constant reference to eventTimes_.
   */
  const scalar_array_t& eventTimes() const { return eventTimes_; }

  /**
   * Retrieves the event times.
   *
   * @return A reference to eventTimes_.
   */
  scalar_array_t& eventTimes() { return eventTimes_; }

  /**
   * Retrieves the number of the event times.
   *
   * @return number of event times.
   */
  size_t getNumEventTimes() const { return eventTimes_.size(); }

  /**
   * Used in the SLQ-MPC method to insert a new user defined logic in the given time period.
   * Note: use the update method to at the end to update your derived class variables
   *
   * @param [in] startTime: The initial time from which the new logicRules template should be augmented.
   * @param [in] finalTime: The final time to which the new logicRules template should be augmented.
   */
  void insertInternalModeSequenceTemplate(const scalar_t& startTime, const scalar_t& finalTime) {
    insertModeSequenceTemplate(modeSequenceTemplate_, startTime, finalTime);
  }

  /**
   * Rewinds the class. This method is only called in the MPC class.
   * Note: use the update method to at the end to update your derived class variables.
   *
   * @param [in] lowerBoundTime: The smallest time for which the logicRules should be defined.
   * @param [in] upperBoundTime: The greatest time for which the logicRules should be defined.
   */
  virtual void rewind(const scalar_t& lowerBoundTime, const scalar_t& upperBoundTime) = 0;

  /**
   * This method can be used to update the internal variables. This method should be called by any
   * program that updates the logic rules variables e.g. rewind,
   */
  virtual void update() = 0;

  /**
   *  Finds the event count based on the query time and eventTimes
   *  Events are counted as follows:
   *  		------ | ----- | ---  ... ---    | -----
   *				   t0     t1              t(n-1)
   *  count     0        1      2   ...  (n-1)   n
   *
   *  If time equal equal to a switch time is requested, the lower count is taken
   *
   *  @param [in] time
   *  @return count of the event the input time belongs to
   */
  size_t getEventTimeCount(scalar_t time) const { return Lookup::findIndexInTimeArray(eventTimes_, time); };

 protected:
  /**
   * Used in the SLQ-MPC method to insert a new user defined logic in the given time period.
   * Note: use the update method to at the end to update your derived class variables
   *
   * @param [in] modeSequenceTemplate: A data type which includes all necessary information for modifying the logicRules.
   * @param [in] startTime: The initial time from which the new logicRules template should be augmented.
   * @param [in] finalTime: The final time to which the new logicRules template should be augmented.
   */
  virtual void insertModeSequenceTemplate(const logic_template_type& modeSequenceTemplate, const scalar_t& startTime,
                                          const scalar_t& finalTime) = 0;

  // Subsystems and eventTimes contain the currently valid modesequence
  size_array_t subsystemsSequence_;
  scalar_array_t eventTimes_;
  // This mode sequence template provides a template to extend the modesequence into the future.
  logic_template_type modeSequenceTemplate_;
};

}  // namespace ocs2

#endif /* HYBRIDLOGICRULES_OCS2_H_ */
