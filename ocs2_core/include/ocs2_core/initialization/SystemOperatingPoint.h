/*
 * SystemOperatingPoint.h
 *
 *  Created on: Dec 11, 2017
 *      Author: farbod
 */

#ifndef SYSTEMOPERATINGPOINT_OCS2_H_
#define SYSTEMOPERATINGPOINT_OCS2_H_

#include <memory>

#include "ocs2_core/initialization/SystemOperatingTrajectoriesBase.h"

namespace ocs2{

/**
 * This is base class for initializing the SLQ-based algorithms.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules<STATE_DIM,INPUT_DIM>>
class SystemOperatingPoint : public SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> Base;

	typedef std::shared_ptr<SystemOperatingPoint<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>> Ptr;

	typedef typename Base::scalar_t 			scalar_t;
	typedef typename Base::scalar_array_t 		scalar_array_t;
	typedef typename Base::size_array_t 		size_array_t;
	typedef typename Base::state_vector_t   	state_vector_t;
	typedef typename Base::state_vector_array_t state_vector_array_t;
	typedef typename Base::input_vector_t		input_vector_t;
	typedef typename Base::input_vector_array_t input_vector_array_t;

	/**
	 * Default constructor
	 */
	SystemOperatingPoint(const state_vector_t& stateOperatingPoint, const input_vector_t& inputOperatingPoint)
	: stateOperatingPoint_(stateOperatingPoint),
	  inputOperatingPoint_(inputOperatingPoint)
	{}

	/**
	 * Default destructor.
	 */
	virtual ~SystemOperatingPoint() {}

	/**
	 * Initializes the operating trajectories class.
	 *
	 * @param [in] logicRulesMachine: A class which contains and parse the logic rules e.g
	 * method findActiveSubsystemHandle returns a Lambda expression which can be used to
	 * find the ID of the current active subsystem.
	 * @param [in] partitionIndex: index of the time partition.
	 * @param [in] algorithmName: The algorithm that class this class (default not defined).
	 */
	virtual void initializeModel(const LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>& logicRulesMachine,
			const size_t& partitionIndex, const char* algorithmName=NULL) override {

		Base::initializeModel(logicRulesMachine, partitionIndex, algorithmName);
	}

	/**
	 * Returns pointer to the class.
	 *
	 * @return A raw pointer to the class.
	 */
	virtual SystemOperatingPoint<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>* clone() const override {
		return new SystemOperatingPoint<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>(*this);
	}

	/**
	 * Gets the Operating ponits for the system in time interval [startTime, finalTime] where there is
	 * no intermediate switches except possibly the end time.
	 *
	 * @param [in] initialState: Initial state.
	 * @param [in] startTime: Initial time.
	 * @param [in] finalTime: Final time.
	 * @param [out] timeTrajectory: Output time stamp trajectory.
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [out] inputTrajectory: Output control input trajectory.
	 * @param [in] concatOutput: Whether to concatenate the output to the input trajectories or
	 * override (default).
	 */
	virtual void getSystemOperatingTrajectories(
			const state_vector_t& initialState,
			const double& startTime,
			const double& finalTime,
			scalar_array_t& timeTrajectory,
			state_vector_array_t& stateTrajectory,
			input_vector_array_t& inputTrajectory,
			bool concatOutput = false) override {

		if (concatOutput==false) {
			timeTrajectory.clear();
			stateTrajectory.clear();
			inputTrajectory.clear();
		}

		timeTrajectory.emplace_back(startTime);
		timeTrajectory.emplace_back(finalTime);

		stateTrajectory.emplace_back(stateOperatingPoint_);
		stateTrajectory.emplace_back(stateOperatingPoint_);

		inputTrajectory.emplace_back(inputOperatingPoint_);
		inputTrajectory.emplace_back(inputOperatingPoint_);
	}


private:
	state_vector_t stateOperatingPoint_;
	input_vector_t inputOperatingPoint_;

};

} // namespace ocs2

#endif /* SYSTEMOPERATINGPOINT_OCS2_H_ */
