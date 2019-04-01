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

#ifndef ROLLOUT_BASE_OCS2_H_
#define ROLLOUT_BASE_OCS2_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <algorithm>
#include <array>
#include <memory>
#include <numeric>
#include <type_traits>
#include <vector>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/OCS2NumericTraits.h>
#include <ocs2_core/control/Controller.h>
#include <ocs2_core/logic/machine/LogicRulesMachine.h>
#include <ocs2_core/logic/rules/LogicRulesBase.h>
#include <ocs2_core/logic/rules/NullLogicRules.h>
#include <ocs2_core/misc/FindActiveIntervalIndex.h>

#include "Rollout_Settings.h"

namespace ocs2 {

/**
 * This class is an interface class for forward rollout of the system dynamics.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T = NullLogicRules>
class RolloutBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static_assert(std::is_base_of<LogicRulesBase, LOGIC_RULES_T>::value, "LOGIC_RULES_T must inherit from LogicRulesBase");

  typedef std::shared_ptr<RolloutBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>> Ptr;

  typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;

  typedef typename DIMENSIONS::size_array_t size_array_t;
  typedef typename DIMENSIONS::scalar_t scalar_t;
  typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
  typedef typename DIMENSIONS::state_vector_t state_vector_t;
  typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
  typedef typename DIMENSIONS::input_vector_t input_vector_t;
  typedef typename DIMENSIONS::input_vector_array_t input_vector_array_t;

  typedef LogicRulesMachine<LOGIC_RULES_T> logic_rules_machine_t;
  typedef Controller<STATE_DIM, INPUT_DIM> controller_t;

  /**
   * Default constructor.
   *
   * @param [in] rolloutSettings: The rollout settings.
   * @param [in] algorithmName: The algorithm that calls this class (default not defined).
   */
  RolloutBase(const Rollout_Settings& rolloutSettings = Rollout_Settings(), const char* algorithmName = NULL)

      : rolloutSettings_(rolloutSettings), algorithmName_(algorithmName) {}

  /**
   * Default destructor.
   */
  virtual ~RolloutBase() = default;

  /**
   * Returns the rollout settings.
   *
   * @return The rollout settings.
   */
  Rollout_Settings& settings() { return rolloutSettings_; }

  /**
   * Returns the algorithm's name which called this class.
   *
   * @return The algorithm's name which called this class.
   */
  const char* algorithmName() { return algorithmName_; }

  /**
   * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
   * to integrate the system dynamics in time period [initTime, finalTime].
   *
   * @param [in] partitionIndex: Time partition index.
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] controller: control policy.
   * @param [in] logicRulesMachine: logic rules machine.
   * @param [out] timeTrajectory: The time trajectory stamp.
   * @param [out] eventsPastTheEndIndeces: Indices containing past-the-end index of events trigger.
   * @param [out] stateTrajectory: The state trajectory.
   * @param [out] inputTrajectory: The control input trajectory.
   * @return The final state (state jump is considered if it took place)
   */
  virtual state_vector_t run(const size_t& partitionIndex, const scalar_t& initTime, const state_vector_t& initState,
                             const scalar_t& finalTime, controller_t* controller, logic_rules_machine_t& logicRulesMachine,
                             scalar_array_t& timeTrajectory, size_array_t& eventsPastTheEndIndeces, state_vector_array_t& stateTrajectory,
                             input_vector_array_t& inputTrajectory) = 0;

  /**
   * Prints out the rollout.
   *
   * @param [in] partitionIndex: Time partition index.
   * @param [in] timeTrajectory: The time trajectory stamp.
   * @param [in] eventsPastTheEndIndeces: Indices containing past-the-end index of events trigger.
   * @param [in] stateTrajectory: The state trajectory.
   * @param [in] inputTrajectory: The control input trajectory.
   */
  static void display(const size_t& partitionIndex, const scalar_array_t& timeTrajectory, const size_array_t& eventsPastTheEndIndeces,
                      const state_vector_array_t& stateTrajectory, const input_vector_array_t& inputTrajectory) {
    std::cerr << std::endl << "++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "Partition: " << partitionIndex;
    std::cerr << std::endl << "++++++++++++++++++++++++++++++" << std::endl;
    std::cerr << "Trajectory length:      " << timeTrajectory.size() << std::endl;
    std::cerr << "Total number of events: " << eventsPastTheEndIndeces.size() << std::endl;
    if (!eventsPastTheEndIndeces.empty()) {
      std::cerr << "Event times: ";
      for (size_t ind : eventsPastTheEndIndeces) std::cerr << timeTrajectory[ind] << ", ";
      std::cerr << std::endl;
    }
    std::cerr << std::endl;

    size_t k = 0;
    for (size_t i = 0; i <= eventsPastTheEndIndeces.size(); i++) {
      for (; k < timeTrajectory.size(); k++) {
        std::cerr << "k:     " << k << std::endl;
        std::cerr << "Time:  " << timeTrajectory[k] << std::endl;
        std::cerr << "State: " << stateTrajectory[k].transpose() << std::endl;
        std::cerr << "Input: " << inputTrajectory[k].transpose() << std::endl;

        if (i < eventsPastTheEndIndeces.size() && k + 1 == eventsPastTheEndIndeces[i]) {
          std::cerr << "+++ event took place +++" << std::endl;
          break;
        }
      }  // end of k loop
    }    // end of i loop
  }

 private:
  Rollout_Settings rolloutSettings_;

  const char* algorithmName_;
};

}  // namespace ocs2

#endif /* ROLLOUT_BASE_OCS2_H_ */
