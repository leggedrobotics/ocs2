/*
 * SystemOperatingTrajectoriesBase.h
 *
 *  Created on: Dec 11, 2017
 *      Author: farbod
 */

#ifndef SYSTEMOPERATINGTRAJECTORIESBASE_OCS2_H_
#define SYSTEMOPERATINGTRAJECTORIESBASE_OCS2_H_

#include <type_traits>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/logic/LogicRulesMachine.h"

namespace ocs2{

/**
 * This is base class for initializing the SLQ-based algorithms.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules<STATE_DIM,INPUT_DIM>>
class SystemOperatingTrajectoriesBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static_assert(std::is_base_of<LogicRulesBase<STATE_DIM, INPUT_DIM, typename LOGIC_RULES_T::LogicRulesTemplate>, LOGIC_RULES_T>::value,
			"LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef std::shared_ptr<SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > Ptr;
	typedef std::shared_ptr<const SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> > ConstPtr;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t 				scalar_t;
	typedef typename DIMENSIONS::scalar_array_t 		scalar_array_t;
	typedef typename DIMENSIONS::size_array_t 			size_array_t;
	typedef typename DIMENSIONS::state_vector_t   		state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t   state_vector_array_t;
	typedef typename DIMENSIONS::control_vector_t 		input_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t input_vector_array_t;

	/**
	 * Default constructor
	 */
	SystemOperatingTrajectoriesBase() {}

	/**
	 * Default destructor.
	 */
	virtual ~SystemOperatingTrajectoriesBase() {}

	/**
	 * Initializes the operating trajectories class.
	 *
	 * @param [in] logicRulesMachine: A class which contains and parse the logic rules e.g
	 * method findActiveSubsystemHandle returns a Lambda expression which can be used to
	 * find the ID of the current active subsystem.
	 * @param [in] partitionIndex: index of the time partition.
	 * @param [in] algorithmName: The algorithm that class this class (default not defined).
	 */
	virtual void initializeModel(LogicRulesMachine<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>& logicRulesMachine,
			const size_t& partitionIndex, const char* algorithmName=NULL)
	{}

	/**
	 * Returns pointer to the class.
	 *
	 * @return A raw pointer to the class.
	 */
	virtual SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>* clone() const = 0;

	/**
	 * Gets the Operating Trajectories of the system in time interval [startTime, finalTime] where there is
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
			const scalar_t& startTime,
			const scalar_t& finalTime,
			scalar_array_t& timeTrajectory,
			state_vector_array_t& stateTrajectory,
			input_vector_array_t& inputTrajectory,
			bool concatOutput = false) = 0;

private:

};

} // namespace ocs2

#endif /* SYSTEMOPERATINGTRAJECTORIESBASE_OCS2_H_ */
