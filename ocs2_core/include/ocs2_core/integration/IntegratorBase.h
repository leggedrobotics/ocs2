/*
 * IntegratorBase.h
 *
 *  Created on: 17.12.2015
 *      Author: farbodf
 */

#ifndef OCS2_INTEGRATORBASE_H_
#define OCS2_INTEGRATORBASE_H_

#include <limits>

#include "ocs2_core/dynamics/SystemBase.h"
#include "ocs2_core/integration/Observer.h"
#include "ocs2_core/integration/EventHandler.h"


namespace ocs2{

/**
 * The interface class for integration of autonomous systems.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 */
template <int STATE_DIM>
class IntegratorBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::vector<double> TimeTrajectory_T;
	typedef Eigen::Matrix<double, STATE_DIM, 1> State_T;
	typedef std::vector<State_T, Eigen::aligned_allocator<State_T> > StateTrajectory_T;

	/**
	 * Constructor
	 * @param [in] system: The system dynamics.
	 * @param [in] eventHandler: The integration event function.
	 */
	IntegratorBase(
			const std::shared_ptr<SystemBase<STATE_DIM> >& system,
			const std::shared_ptr<EventHandler<STATE_DIM> >& eventHandler = nullptr)

	: observer_(eventHandler),
	  system_(system),
	  eventHandler_(eventHandler)
	{}

	/**
	 * Default destructor
	 */
	virtual ~IntegratorBase() {}

	/**
	 * Equidistant integration based on initial and final time as well as step length
	 * @param [in] initialState: Initial state.
	 * @param [in] startTime: Initial time.
	 * @param [in] finalTime: Final time.
	 * @param [in] dt: Time step.
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [out] timeTrajectory: Output time stamp trajectory.
	 * @param [in] concatOutput: Whether to concatenate the output to the input trajectories or override (default).
	 */
	virtual void integrate(
			const State_T& initialState,
			const double& startTime,
			const double& finalTime,
			double dt,
			StateTrajectory_T& stateTrajectory,
			TimeTrajectory_T& timeTrajectory,
			bool concatOutput = false
	) = 0;

	/**
	 * Adaptive time integration based on start time and final time. This method can solve ODEs with time-dependent events,
	 * if eventsTime is not empty. In this case the output time-trajectory contains two identical values at the moments
	 * of event triggerings. This method uses SystemBase::mapState() method for state transition at events.
	 *
	 * @param [in] initialState: Initial state.
	 * @param [in] startTime: Initial time.
	 * @param [in] finalTime: Final time.
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [out] timeTrajectory: Output time stamp trajectory.
	 * @param [in] dtInitial: Initial time step.
	 * @param [in] AbsTol: The absolute tolerance error for ode solver.
	 * @param [in] RelTol: The relative tolerance error for ode solver.
	 * @param [in] maxNumSteps: The maximum number of integration points per a second for ode solver.
	 * @param [in] concatOutput: Whether to concatenate the output to the input trajectories or override (default).
	 */
	virtual void integrate(
			const State_T& initialState,
			const double& startTime,
			const double& finalTime,
			StateTrajectory_T& stateTrajectory,
			TimeTrajectory_T& timeTrajectory,
			double dtInitial = 0.01,
			double AbsTol = 1e-6,
			double RelTol = 1e-3,
			size_t maxNumSteps = std::numeric_limits<size_t>::max(),
			bool concatOutput = false
	) = 0;


	/**
	 * Output integration based on a given time trajectory. This method can solve ODEs with time-dependent events.
	 * In this case, user should pass past-the-end indeces of events on the input time trajectory. Moreover, this
	 * method assumes that there are two identical time values in the input time-trajectory at the moments of event
	 * triggerings. This method uses SystemBase::mapState() method for state transition at events.
	 *
	 * @param [in] initialState: Initial state.
	 * @param [in] beginTimeItr: The iterator to the begining of the time stamp trajectory.
	 * @param [in] endTimeItr: The iterator to the end of the time stamp trajectory.
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [in] dtInitial: Initial time step.
	 * @param [in] AbsTol: The absolute tolerance error for ode solver.
	 * @param [in] RelTol: The relative tolerance error for ode solver.
	 * @param [in] maxNumSteps: The maximum number of integration points per a second for ode solver.
	 * @param [in] concatOutput: Whether to concatenate the output to the input trajectories or override (default).
	 */
	virtual void integrate(
			const State_T& initialState,
			typename TimeTrajectory_T::const_iterator beginTimeItr,
			typename TimeTrajectory_T::const_iterator endTimeItr,
			StateTrajectory_T& stateTrajectory,
			double dtInitial = 0.01,
			double AbsTol = 1e-9,
			double RelTol = 1e-6,
			size_t maxNumSteps = std::numeric_limits<size_t>::max(),
			bool concatOutput = false
	) = 0;


protected:

	/**
	 * Set state and time trajectory pointer to observer.
	 *
	 * @param stateTrajectory: Output state trajectory.
	 * @param timeTrajectory: Output time stamp trajectory.
	 */
	void setOutputTrajectoryPtrToObserver(StateTrajectory_T* stateTrajectoryPtr,
			TimeTrajectory_T* timeTrajectoryPtr = nullptr) {
		observer_.setStateTrajectory(stateTrajectoryPtr);
		if (timeTrajectoryPtr) {
			observer_.setTimeTrajectory(timeTrajectoryPtr);
		} else {
			tempTimeTrajectory_.resize(stateTrajectoryPtr->size());
			observer_.setTimeTrajectory(&tempTimeTrajectory_);
		}
	}

	/**
	 * Set time trajectory pointer to observer.
	 *
	 * @param timeTrajectory: Output time stamp trajectory.
	 */
	void setTimeTrajectoryPtrToObserver(TimeTrajectory_T& timeTrajectory)
	{
		observer_.setTimeTrajectory(&timeTrajectory);
	}

	/**
	 * Set state trajectory pointer to observer.
	 *
	 * @param stateTrajectory: Output state trajectory.
	 */
	void setStateTrajectoryPtrToObserver(StateTrajectory_T& stateTrajectory)
	{
		observer_.setStateTrajectory(&stateTrajectory);
	}

	/**
	 * Integrator observer for saving the variable of interest.
	 */
	Observer<STATE_DIM> observer_;

	/**
	 * System dynamics used by integrator.
	 */
	std::shared_ptr<SystemBase<STATE_DIM> > system_;

	/**
	 * Event handler used by integrator.
	 */
	std::shared_ptr<EventHandler<STATE_DIM> > eventHandler_;

private:
	std::vector<double> tempTimeTrajectory_; // used for the output integration case based on a given time trajectory.
};

} // namespace ocs2

#endif /* OCS2INTEGRATORBASE_H_ */
