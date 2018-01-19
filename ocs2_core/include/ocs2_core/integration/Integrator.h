/*
 *
 * Integrator.h
 *
 *  Created on: 17.12.2015
 *      Author: farbodf
 *
 */

#ifndef OCS2_INTEGRATOR_H_
#define OCS2_INTEGRATOR_H_

#include <type_traits>
#include <functional>
#include <cmath>

#include <boost/numeric/odeint.hpp>

#include "ocs2_core/OCS2NumericTraits.h"
#include "ocs2_core/integration/eigenIntegration.h"
#include "ocs2_core/integration/IntegratorBase.h"

namespace ocs2{


/**
 * Euler steppers
 */
template <int STATE_DIM>
using euler_t = boost::numeric::odeint::euler<
		Eigen::Matrix<double, STATE_DIM, 1>,
		double,
		Eigen::Matrix<double, STATE_DIM, 1>,
		double,
		boost::numeric::odeint::vector_space_algebra >;

/**
 * Modified_Midpoint steppers
 */
template <int STATE_DIM>
using modified_midpoint_t = boost::numeric::odeint::modified_midpoint<
		Eigen::Matrix<double, STATE_DIM, 1>,
		double,
		Eigen::Matrix<double, STATE_DIM, 1>,
		double,
		boost::numeric::odeint::vector_space_algebra >;

/**
 * 4th order Runge_Kutta steppers
 */
template <int STATE_DIM>
using runge_kutta_4_t = boost::numeric::odeint::runge_kutta4<
		Eigen::Matrix<double, STATE_DIM, 1>,
		double,
		Eigen::Matrix<double, STATE_DIM, 1>,
		double,
		boost::numeric::odeint::vector_space_algebra >;

/**
 * 5th order Runge_Kutta_Dopri steppers
 */
template <int STATE_DIM>
using runge_kutta_dopri5_t = boost::numeric::odeint::runge_kutta_dopri5 <
		Eigen::Matrix<double, STATE_DIM, 1>,
		double,
		Eigen::Matrix<double, STATE_DIM, 1>,
		double,
		boost::numeric::odeint::vector_space_algebra>;

/**
 * Dense_output Runge_Kutta steppers
 */
template <int STATE_DIM>
using dense_runge_kutta5_t = boost::numeric::odeint::dense_output_runge_kutta <
		boost::numeric::odeint::controlled_runge_kutta <runge_kutta_dopri5_t<STATE_DIM>> >;

/**
 * Bulirsch_Stoer steppers
 */
template <int STATE_DIM>
using bulirsch_stoer_t = boost::numeric::odeint::bulirsch_stoer <
		Eigen::Matrix<double, STATE_DIM, 1>,
		double,
		Eigen::Matrix<double, STATE_DIM, 1>,
		double,
		boost::numeric::odeint::vector_space_algebra>;

/**
 * Adams_Bashforth steppers
 */
template <int STATE_DIM, size_t STEPS>
using adams_bashforth_uncontrolled_t =
		boost::numeric::odeint::adams_bashforth<
		STEPS,
		Eigen::Matrix<double, STATE_DIM, 1>,	// state
		double,									// typename value
		Eigen::Matrix<double, STATE_DIM, 1>,	// derivative
		double, 								// typename time
		boost::numeric::odeint::vector_space_algebra> ;

// works only for boost 1.56 or higher
//template <int STATE_DIM, size_t STEPS>
//using adams_bashforth_moulton_uncontrolled_t =
//		boost::numeric::odeint::adams_bashforth_moulton<
//		STEPS,
//		Eigen::Matrix<double, STATE_DIM, 1>,	// state
//		double,									// typename value
//		Eigen::Matrix<double, STATE_DIM, 1>,	// derivative
//		double, 								// typename time
//		boost::numeric::odeint::vector_space_algebra> ;

/**
 * Integrator class for autonomous systems.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam Stepper: Stepper class type to be used.
 */
template <int STATE_DIM, class Stepper>
class Integrator : public IntegratorBase<STATE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef IntegratorBase<STATE_DIM> Base;

	/**
	 * Constructor
	 * @param [in] system: The system dynamics.
	 * @param [in] eventHandler: The integration event function.
	 */
	Integrator(
			const std::shared_ptr<SystemBase<STATE_DIM> >& system,
			const std::shared_ptr<EventHandler<STATE_DIM> >& eventHandler = nullptr)

	: Base(system, eventHandler)
	{
		setupSystem();
	}

	/**
	 * Equidistant integration based on initial and final time as well as step length
	 * @param [in] initialState: Initial state.
	 * @param [in] startTime: Initial time.
	 * @param [in] finalTime: Final time.
	 * @param [in] dt: Time step.
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [out] timeTrajectory: Output time stamp trajectory.
	 * @param [in] Whether to concatenate the output to the input trajectories or override (default).
	 */
	void integrate(
			const typename Base::State_T& initialState,
			const double& startTime,
			const double& finalTime,
			double dt,
			typename Base::StateTrajectory_T& stateTrajectory,
			typename Base::TimeTrajectory_T& timeTrajectory,
			bool concatOutput = false) override {

		typename Base::State_T initialStateInternal = initialState;

		/*
		 * use a temporary state for initialization, the state returned by initialize is different
		 * from the real init state (already forward integrated)
		 */
		typename Base::State_T initialStateInternal_init_temp = initialState;

		double startTime_temp = startTime;

		// reset the trajectories
		if (concatOutput==false) {
			timeTrajectory->clear();
			stateTrajectory->clear();
		}

		Base::setTimeTrajectoryPtrToObserver(timeTrajectory);
		Base::setStateTrajectoryPtrToObserver(stateTrajectory);

		initialize(initialStateInternal_init_temp, startTime_temp, dt);

		boost::numeric::odeint::integrate_const(stepper_, systemFunction_,
				initialStateInternal, startTime, finalTime+0.1*dt, dt, Base::observer_.observeWrap);
	}

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
	 * @param [in] Whether to concatenate the output to the input trajectories or override (default).
	 */
	void integrate(const typename Base::State_T& initialState,
			const double& startTime,
			const double& finalTime,
			typename Base::StateTrajectory_T& stateTrajectory,
			typename Base::TimeTrajectory_T& timeTrajectory,
			double dtInitial = 0.01,
			double AbsTol = 1e-6,
			double RelTol = 1e-3,
			size_t maxNumSteps = std::numeric_limits<size_t>::max(),
			bool concatOutput = false)  override  {

		typename Base::State_T internalStartState = initialState;

		if (maxNumSteps < std::numeric_limits<size_t>::max())
			Base::observer_.setMaxNumSteps(maxNumSteps, Base::system_);

		// reset the trajectories
		if (concatOutput==false) {
			timeTrajectory->clear();
			stateTrajectory->clear();
		}

		Base::setTimeTrajectoryPtrToObserver(timeTrajectory);
		Base::setStateTrajectoryPtrToObserver(stateTrajectory);

		integrate_adaptive_specialized<Stepper>(internalStartState, startTime, finalTime, dtInitial, AbsTol, RelTol);
	}

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
	 * @param [in] Whether to concatenate the output to the input trajectories or override (default).
	 */
	void integrate(const typename Base::State_T& initialState,
			typename Base::TimeTrajectory_T::const_iterator beginTimeItr,
			typename Base::TimeTrajectory_T::const_iterator endTimeItr,
			typename Base::StateTrajectory_T& stateTrajectory,
			double dtInitial = 0.01,
			double AbsTol = 1e-9,
			double RelTol = 1e-6,
			size_t maxNumSteps = std::numeric_limits<size_t>::max(),
			bool concatOutput = false) override  {

		typename Base::State_T internalStartState = initialState;

		if (maxNumSteps < std::numeric_limits<size_t>::max())
			Base::observer_.setMaxNumSteps(maxNumSteps, Base::system_);

		// reset the trajectories
		if (concatOutput==false) {
			stateTrajectory->clear();
		}

		Base::setStateTrajectoryPtrToObserver(stateTrajectory);

		integrate_times_specialized<Stepper>(internalStartState, beginTimeItr, endTimeItr, dtInitial, AbsTol, RelTol);
	}


private:

	/**
	 * Setup System
	 */
	void setupSystem()
	{
		systemFunction_ = [this]( const Eigen::Matrix<double, STATE_DIM, 1>& x, Eigen::Matrix<double, STATE_DIM, 1>& dxdt, double t ){
			const typename Base::State_T& xState(static_cast<const typename Base::State_T& >(x));
			typename Base::State_T& dxdtState(static_cast<typename Base::State_T& >(dxdt));
			this->system_->computeDerivative(t, xState, dxdtState);
		};
	}

	/**
	 * Intialize
	 * @param [in] initialState
	 * @param [in] t
	 * @param [in] dt
	 */
	void initialize(typename Base::State_T& initialState, double& t, double dt)
	{
//		initializeStepper(initialState, t, dt);	// TODO
	}


	/**
	 * Integrate adaptive specialized.
	 *
	 * @tparam S: stepper type.
	 * @param [in] initialState: Initial state.
	 * @param [in] startTime: Initial time.
	 * @param [in] finalTime: Final time.
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [out] timeTrajectory: Output time stamp trajectory.
	 * @param [in] dtInitial: Initial time step.
	 * @param [in] AbsTol: The absolute tolerance error for ode solver.
	 * @param [in] RelTol: The relative tolerance error for ode solver.
	 * @param [in] maxNumSteps: The maximum number of integration points per a second for ode solver.
	 * @return
	 */
	template <typename S>
	typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
	integrate_adaptive_specialized(
			typename Base::State_T& initialState,
			const double& startTime,
			const double& finalTime,
			double dtInitial,
			double AbsTol,
			double RelTol) {

		boost::numeric::odeint::integrate_adaptive(boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol), systemFunction_,
				initialState, startTime, finalTime, dtInitial, Base::observer_.observeWrap);
	}

	/**
	 * Integrate adaptive specialized,
	 *
	 * @tparam S: stepper type.
	 * @param [in] initialState: Initial state.
	 * @param [in] startTime: Initial time.
	 * @param [in] finalTime: Final time.
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [out] timeTrajectory: Output time stamp trajectory.
	 * @param [in] dtInitial: Initial time step.
	 * @param [in] AbsTol: The absolute tolerance error for ode solver.
	 * @param [in] RelTol: The relative tolerance error for ode solver.
	 * @param [in] maxNumSteps: The maximum number of integration points per a second for ode solver.
	 * @return
	 */
	template <typename S>
	typename std::enable_if<!std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
	integrate_adaptive_specialized(
			typename Base::State_T& initialState,
			const double& startTime,
			const double& finalTime,
			double dtInitial,
			double AbsTol,
			double RelTol) {

		boost::numeric::odeint::integrate_adaptive(stepper_, systemFunction_,
				initialState, startTime, finalTime, dtInitial, Base::observer_.observeWrap);
	}

	/**
	 * Integrate times specialized function
	 * @tparam S: stepper type.
	 * @param [in] initialState: Initial state.
	 * @param [in] beginTimeItr: The iterator to the begining of the time stamp trajectory.
	 * @param [in] endTimeItr: The iterator to the end of the time stamp trajectory.
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [in] dtInitial: Initial time step.
	 * @param [in] AbsTol: The absolute tolerance error for ode solver.
	 * @param [in] RelTol: The relative tolerance error for ode solver.
	 * @return
	 */
	template <typename S = Stepper>
	typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
	integrate_times_specialized(
			typename Base::State_T& initialState,
			typename Base::TimeTrajectory_T::const_iterator beginTimeItr,
			typename Base::TimeTrajectory_T::const_iterator endTimeItr,
			double dtInitial,
			double AbsTol,
			double RelTol){

		boost::numeric::odeint::integrate_times(boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol), systemFunction_,
				initialState, beginTimeItr, endTimeItr, dtInitial, Base::observer_.observeWrap);
	}

	/**
	 * Integrate times specialized function
	 * @tparam S: stepper type.
	 * @param [in] initialState: Initial state.
	 * @param [in] beginTimeItr: The iterator to the begining of the time stamp trajectory.
	 * @param [in] endTimeItr: The iterator to the end of the time stamp trajectory.
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [in] dtInitial: Initial time step.
	 * @param [in] AbsTol: The absolute tolerance error for ode solver.
	 * @param [in] RelTol: The relative tolerance error for ode solver.
	 * @return
	 */
	template <typename S = Stepper>
	typename std::enable_if<!std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
	integrate_times_specialized(
			typename Base::State_T& initialState,
			typename Base::TimeTrajectory_T::const_iterator beginTimeItr,
			typename Base::TimeTrajectory_T::const_iterator endTimeItr,
			double dtInitial,
			double AbsTol,
			double RelTol){

		boost::numeric::odeint::integrate_times(stepper_, systemFunction_,
				initialState, beginTimeItr, endTimeItr, dtInitial, Base::observer_.observeWrap);
	}


	/**
	 * Functionality to reset stepper. If we integrate with ODE45, we don't need to reset the stepper, hence specialize empty function
	 * @tparam S: stepper type.
	 * @param [in] initialState: Initial state.
	 * @param [in] t: Time.
	 * @param [in] dt: Time step.
	 * @return
	 */
	template <typename S = Stepper>
	typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
	initializeStepper(typename Base::State_T& initialState, double& t, double dt)
	{
		/**do nothing, runge_kutta_5_t does not have a init method */
	}

	/**
	 * Functionality to reset stepper. If we integrate with some other method, eg. adams_bashforth, we need to reset the stepper, hence specialize with initialize call
	 * @tparam S
	 * @param [in] initialState
	 * @param [in] t: Time.
	 * @param [in] dt: Time step.
	 * @return
	 */
	template <typename S = Stepper>
	typename std::enable_if<!(std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value), void>::type
	initializeStepper(typename Base::State_T& initialState, double& t, double dt)
	{
		stepper_.initialize(runge_kutta_dopri5_t<STATE_DIM>(), systemFunction_, initialState, t, dt);
	}


	std::function<void (const Eigen::Matrix<double, STATE_DIM, 1>&, Eigen::Matrix<double, STATE_DIM, 1>&, double)> systemFunction_;

	Stepper stepper_;
};


/**
 * Euler integrator.
 */
template <int STATE_DIM>
using IntegratorEuler = Integrator<STATE_DIM, euler_t<STATE_DIM>>;

/**
 * Modified midpoint integrator.
 */
template <int STATE_DIM>
using IntegratorModifiedMidpoint = Integrator<STATE_DIM, modified_midpoint_t<STATE_DIM>>;

/**
 * RK4 integrator.
 */
template <int STATE_DIM>
using IntegratorRK4 = Integrator<STATE_DIM, runge_kutta_4_t<STATE_DIM>>;

/**
 * RK5 variable integrator.
 */
template <int STATE_DIM>
using IntegratorRK5Variable = Integrator<STATE_DIM, dense_runge_kutta5_t<STATE_DIM>>;

/**
 * ode45 integrator.
 */
template <int STATE_DIM>
using ODE45 = Integrator<STATE_DIM, runge_kutta_dopri5_t<STATE_DIM>>;

/**
 * Adams-Bashforth integrator.
 */
template <int STATE_DIM, size_t STEPS>
using IntegratorAdamsBashforth = Integrator < STATE_DIM, adams_bashforth_uncontrolled_t<STATE_DIM, STEPS>>;

/**
 * Bulirsch-Stoer integrator.
 */
template <int STATE_DIM>
using IntegratorBulirschStoer = Integrator < STATE_DIM, bulirsch_stoer_t<STATE_DIM>>;


//// works only after boost 1.56
//template <int STATE_DIM, size_t STEPS>
//using IntegratorAdamsBashforthMoulton = Integrator < STATE_DIM, adams_bashforth_moulton_uncontrolled_t<STATE_DIM, STEPS>>;

} // namespace ocs2

#endif /* OCS2INTEGRATOR_H_ */
