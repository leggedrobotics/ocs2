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

	typedef IntegratorBase<STATE_DIM> BASE;
	typedef typename BASE::scalar_t				scalar_t;
	typedef typename BASE::scalar_array_t 		scalar_array_t;
	typedef typename BASE::state_vector_t		state_vector_t;
	typedef typename BASE::state_vector_array_t state_vector_array_t;

	/**
	 * Constructor
	 * @param [in] system: The system dynamics.
	 * @param [in] eventHandler: The integration event function.
	 */
	Integrator(
			const std::shared_ptr<SystemBase<STATE_DIM> >& systemPtr,
			const std::shared_ptr<SystemEventHandler<STATE_DIM> >& eventHandlerPtr = nullptr)

	: BASE(systemPtr, eventHandlerPtr)
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
	 * @param [in] concatOutput: Whether to concatenate the output to the input trajectories or override (default).
	 */
	void integrate(
			const state_vector_t& initialState,
			const scalar_t& startTime,
			const scalar_t& finalTime,
			scalar_t dt,
			state_vector_array_t& stateTrajectory,
			scalar_array_t& timeTrajectory,
			bool concatOutput = false) override {

		state_vector_t initialStateInternal = initialState;

		/*
		 * use a temporary state for initialization, the state returned by initialize is different
		 * from the real init state (already forward integrated)
		 */
		state_vector_t initialStateInternal_init_temp = initialState;

		scalar_t startTime_temp = startTime;

		// reset the trajectories
		if (concatOutput==false) {
			timeTrajectory.clear();
			stateTrajectory.clear();
		}

		BASE::setOutputTrajectoryPtrToObserver(&stateTrajectory, &timeTrajectory);

		initialize(initialStateInternal_init_temp, startTime_temp, dt);

		boost::numeric::odeint::integrate_const(stepper_, systemFunction_,
				initialStateInternal, startTime, finalTime+0.1*dt, dt, BASE::observer_.observeWrap);
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
	 * @param [in] concatOutput: Whether to concatenate the output to the input trajectories or override (default).
	 */
	void integrate(
			const state_vector_t& initialState,
			const scalar_t& startTime,
			const scalar_t& finalTime,
			state_vector_array_t& stateTrajectory,
			scalar_array_t& timeTrajectory,
			scalar_t dtInitial = 0.01,
			scalar_t AbsTol = 1e-6,
			scalar_t RelTol = 1e-3,
			size_t maxNumSteps = std::numeric_limits<size_t>::max(),
			bool concatOutput = false)  override  {

		state_vector_t internalStartState = initialState;

		if (BASE::eventHandlerPtr_ && maxNumSteps<std::numeric_limits<size_t>::max())
			BASE::eventHandlerPtr_->setMaxNumSteps(maxNumSteps);

		// reset the trajectories
		if (concatOutput==false) {
			timeTrajectory.clear();
			stateTrajectory.clear();
		}

		BASE::setOutputTrajectoryPtrToObserver(&stateTrajectory, &timeTrajectory);

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
	 * @param [in] concatOutput: Whether to concatenate the output to the input trajectories or override (default).
	 */
	void integrate(const state_vector_t& initialState,
			typename scalar_array_t::const_iterator beginTimeItr,
			typename scalar_array_t::const_iterator endTimeItr,
			state_vector_array_t& stateTrajectory,
			scalar_t dtInitial = 0.01,
			scalar_t AbsTol = 1e-9,
			scalar_t RelTol = 1e-6,
			size_t maxNumSteps = std::numeric_limits<size_t>::max(),
			bool concatOutput = false) override  {

		state_vector_t internalStartState = initialState;

		if (BASE::eventHandlerPtr_ && maxNumSteps<std::numeric_limits<size_t>::max())
			BASE::eventHandlerPtr_->setMaxNumSteps(maxNumSteps);

		// reset the trajectories
		if (concatOutput==false) {
			stateTrajectory.clear();
		}

		BASE::setOutputTrajectoryPtrToObserver(&stateTrajectory);

		integrate_times_specialized<Stepper>(internalStartState, beginTimeItr, endTimeItr, dtInitial, AbsTol, RelTol);
	}


private:

	/**
	 * Setup System
	 */
	void setupSystem()
	{
		systemFunction_ = [this]( const Eigen::Matrix<scalar_t, STATE_DIM, 1>& x, Eigen::Matrix<scalar_t, STATE_DIM, 1>& dxdt, scalar_t t ){
			const state_vector_t& xState(static_cast<const state_vector_t& >(x));
			state_vector_t& dxdtState(static_cast<state_vector_t& >(dxdt));
			this->systemPtr_->computeDerivative(t, xState, dxdtState);
		};
	}

	/**
	 * Intialize
	 * @param [in] initialState
	 * @param [in] t
	 * @param [in] dt
	 */
	void initialize(state_vector_t& initialState, scalar_t& t, scalar_t dt)
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
			state_vector_t& initialState,
			const scalar_t& startTime,
			const scalar_t& finalTime,
			scalar_t dtInitial,
			scalar_t AbsTol,
			scalar_t RelTol) {

		boost::numeric::odeint::integrate_adaptive(boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol), systemFunction_,
				initialState, startTime, finalTime, dtInitial, BASE::observer_.observeWrap);
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
			state_vector_t& initialState,
			const scalar_t& startTime,
			const scalar_t& finalTime,
			scalar_t dtInitial,
			scalar_t AbsTol,
			scalar_t RelTol) {

		boost::numeric::odeint::integrate_adaptive(stepper_, systemFunction_,
				initialState, startTime, finalTime, dtInitial, BASE::observer_.observeWrap);
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
			state_vector_t& initialState,
			typename scalar_array_t::const_iterator beginTimeItr,
			typename scalar_array_t::const_iterator endTimeItr,
			scalar_t dtInitial,
			scalar_t AbsTol,
			scalar_t RelTol){

		boost::numeric::odeint::integrate_times(boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol), systemFunction_,
				initialState, beginTimeItr, endTimeItr, dtInitial, BASE::observer_.observeWrap);
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
			state_vector_t& initialState,
			typename scalar_array_t::const_iterator beginTimeItr,
			typename scalar_array_t::const_iterator endTimeItr,
			scalar_t dtInitial,
			scalar_t AbsTol,
			scalar_t RelTol){

		boost::numeric::odeint::integrate_times(stepper_, systemFunction_,
				initialState, beginTimeItr, endTimeItr, dtInitial, BASE::observer_.observeWrap);
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
	initializeStepper(state_vector_t& initialState, scalar_t& t, scalar_t dt)
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
	initializeStepper(state_vector_t& initialState, scalar_t& t, scalar_t dt)
	{
		stepper_.initialize(runge_kutta_dopri5_t<STATE_DIM>(), systemFunction_, initialState, t, dt);
	}


	std::function<void (const Eigen::Matrix<scalar_t, STATE_DIM, 1>&, Eigen::Matrix<scalar_t, STATE_DIM, 1>&, scalar_t)> systemFunction_;

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


/**
 * Adams-Bashforth-Moulton integrator (works only after boost 1.56)
 */
#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)

template <int STATE_DIM, size_t STEPS>
using IntegratorAdamsBashforthMoulton = Integrator < STATE_DIM, adams_bashforth_moulton_uncontrolled_t<STATE_DIM, STEPS>>;

#endif

} // namespace ocs2

#endif /* OCS2INTEGRATOR_H_ */
