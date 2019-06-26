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

#ifndef OPERATINGTRAJECTORIES_ROLLOUT_OCS2_H_
#define OPERATINGTRAJECTORIES_ROLLOUT_OCS2_H_

#include <memory>

#include <ocs2_core/initialization/SystemOperatingTrajectoriesBase.h>

#include "RolloutBase.h"

namespace ocs2 {

/**
 * This class is an interface class for forward rollout of the system dynamics.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules>
class OperatingTrajectoriesRollout : public RolloutBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static_assert(std::is_base_of<LogicRulesBase, LOGIC_RULES_T>::value,
			"LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef RolloutBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> BASE;

	using controller_t = typename BASE::controller_t;
	using size_array_t = typename BASE::size_array_t;
	using scalar_t = typename BASE::scalar_t;
	using scalar_array_t = typename BASE::scalar_array_t;
	using state_vector_t = typename BASE::state_vector_t;
	using state_vector_array_t = typename BASE::state_vector_array_t;
	using input_vector_t = typename BASE::input_vector_t;
	using input_vector_array_t = typename BASE::input_vector_array_t;

	typedef SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> operating_trajectories_t;

	using logic_rules_machine_t = LogicRulesMachine<LOGIC_RULES_T>;

	/**
	 * Constructor.
	 *
	 * @param [in] operatingTrajectories: The operating trajectories used for initialization.
	 * @param [in] rolloutSettings: The rollout settings.
	 * @param [in] algorithmName: The algorithm that calls this class (default not defined).
	 */
	OperatingTrajectoriesRollout(
			const operating_trajectories_t& operatingTrajectories,
			const Rollout_Settings& rolloutSettings = Rollout_Settings(),
			const char* algorithmName = nullptr)

	: BASE(rolloutSettings, algorithmName)
	, operatingTrajectoriesPtr_(operatingTrajectories.clone())
	{}

	/**
	 * Default destructor.
	 */
	~OperatingTrajectoriesRollout() = default;

	/**
	 * Getting the operating trajectories for the time period [initTime, finalTime] with
	 * user defined operating trajectories.
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
	 *
	 * @return The final state (state jump is considered if it took place)
	 */
	state_vector_t run(
			const size_t& partitionIndex,
			const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			controller_t* controller,
			logic_rules_machine_t& logicRulesMachine,
			scalar_array_t& timeTrajectory,
			size_array_t& eventsPastTheEndIndeces,
			state_vector_array_t& stateTrajectory,
			input_vector_array_t& inputTrajectory) override {

		if (initTime > finalTime)
			throw std::runtime_error("Initial time should be less-equal to final time.");

		if (controller != nullptr)
			throw std::runtime_error("Incorrect usage of Operating trajectory; A controller is available.");

		const size_t numEvents = logicRulesMachine.getNumEvents(partitionIndex);
		const size_t numSubsystems = logicRulesMachine.getNumEventCounters(partitionIndex);
		const scalar_array_t& switchingTimes = logicRulesMachine.getSwitchingTimes(partitionIndex);

		// index of the first subsystem
		size_t beginItr = findActiveIntervalIndex(switchingTimes, initTime, 0);
		// index of the last subsystem
		size_t finalItr = findActiveIntervalIndex(switchingTimes, finalTime, numSubsystems-1);

		// clearing the output trajectories
		timeTrajectory.clear();
		timeTrajectory.reserve(2*numSubsystems);
		stateTrajectory.clear();
		stateTrajectory.reserve(2*numSubsystems);
		inputTrajectory.clear();
		inputTrajectory.reserve(2*numSubsystems);
		eventsPastTheEndIndeces.clear();
		eventsPastTheEndIndeces.reserve(2*numSubsystems);

		// initialize operatingTrajectories
		operatingTrajectoriesPtr_->initializeModel(
				logicRulesMachine,
				partitionIndex,
				BASE::algorithmName());

		state_vector_t beginState = initState;
		scalar_t beginTime, endTime;
		for (size_t i=beginItr; i<=finalItr; i++) {

			beginTime = i==beginItr ? initTime  : switchingTimes[i];
			endTime   = i==finalItr ? finalTime : switchingTimes[i+1];

			// get operating trajectories
			operatingTrajectoriesPtr_->getSystemOperatingTrajectories(
					beginState, beginTime, endTime,
					timeTrajectory,
					stateTrajectory,
					inputTrajectory,
					true);

			if (i<finalItr) {
				eventsPastTheEndIndeces.push_back( stateTrajectory.size() );
				beginState = stateTrajectory.back();
			}

		}  // end of i loop

		// If an event has happened at the final time push it to the eventsPastTheEndIndeces
		// numEvents>finalItr means that there the final active subsystem is before an event time.
		// Note: we don't push the state because the input is not yet defined since the next control
		// policy is available)
		bool eventAtFinalTime = numEvents>finalItr &&
				logicRulesMachine.getEventTimes(partitionIndex)[finalItr]<finalTime+OCS2NumericTraits<scalar_t>::limit_epsilon();

		if (eventAtFinalTime) {
			eventsPastTheEndIndeces.push_back( stateTrajectory.size() );
		}

		return stateTrajectory.back();
	}

private:
	std::unique_ptr<operating_trajectories_t> operatingTrajectoriesPtr_;

};

} // namespace ocs2

#endif /* OPERATINGTRAJECTORIES_ROLLOUT_OCS2_H_ */
