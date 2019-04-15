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

#ifndef INTEGRATOR_OCS2_H_
#define INTEGRATOR_OCS2_H_

#include <type_traits>
#include <functional>
#include <cmath>

#include <boost/numeric/odeint.hpp>

#include "ocs2_core/OCS2NumericTraits.h"
#include "ocs2_core/integration/eigenIntegration.h"
#include "ocs2_core/integration/IntegratorBase.h"
#include "ocs2_core/integration/steppers.h"

namespace ocs2 {

/**
 * @brief The IntegratorType enum
 * Enum used in selecting a specific integrator.
 */
enum class IntegratorType { EULER, MODIFIED_MIDPOINT, RK4, RK5_VARIABLE, ODE45, ADAMS_BASHFORTH, BULIRSCH_STOER, ADAMS_BASHFORTH_MOULTON };

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
	 *
	 * @param [in] system: The system dynamics.
	 * @param [in] eventHandler: The integration event function.
	 */
	Integrator(
			const std::shared_ptr<ODE_Base<STATE_DIM> >& systemPtr,
			const std::shared_ptr<SystemEventHandler<STATE_DIM> >& eventHandlerPtr = nullptr);

	/**
	 * Destructor
	 */
	~Integrator() = default;

	/**
	 * Equidistant integration based on initial and final time as well as step length.
	 *
	 * @param [in] initialState: Initial state.
	 * @param [in] startTime: Initial time.
	 * @param [in] finalTime: Final time.
	 * @param [in] dt: Time step.
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [out] timeTrajectory: Output time stamp trajectory.
	 * @param [in] concatOutput: Whether to concatenate the output to the input
	 * trajectories or override (default).
	 */
	void integrate(
			const state_vector_t& initialState,
			const scalar_t& startTime,
			const scalar_t& finalTime,
			scalar_t dt,
			state_vector_array_t& stateTrajectory,
			scalar_array_t& timeTrajectory,
			bool concatOutput = false) final;

	/**
	 * Adaptive time integration based on start time and final time. This method can
	 * solve ODEs with time-dependent events, if eventsTime is not empty. In this case
	 * the output time-trajectory contains two identical values at the moments
	 * of event triggers. This method uses ODE_Base::computeJumpMap() method for
	 * state transition at events.
	 *
	 * @param [in] initialState: Initial state.
	 * @param [in] startTime: Initial time.
	 * @param [in] finalTime: Final time.
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [out] timeTrajectory: Output time stamp trajectory.
	 * @param [in] dtInitial: Initial time step.
	 * @param [in] AbsTol: The absolute tolerance error for ode solver.
	 * @param [in] RelTol: The relative tolerance error for ode solver.
	 * @param [in] maxNumSteps: The maximum number of integration points per a
	 * second for ode solver.
	 * @param [in] concatOutput: Whether to concatenate the output to the input
	 * trajectories or override (default).
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
			int maxNumSteps = std::numeric_limits<int>::max(),
			bool concatOutput = false) final;

	/**
	 * Output integration based on a given time trajectory. This method can solve ODEs
	 * with time-dependent events. In this case, user should pass past-the-end indices
	 * of events on the input time trajectory. Moreover, this method assumes that there
	 * are two identical time values in the input time-trajectory at the moments of event
	 * triggers. This method uses ODE_Base::computeJumpMap() method for state
	 * transition at events.
	 *
	 * @param [in] initialState: Initial state.
	 * @param [in] beginTimeItr: The iterator to the beginning of the time stamp trajectory.
	 * @param [in] endTimeItr: The iterator to the end of the time stamp trajectory.
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [in] dtInitial: Initial time step.
	 * @param [in] AbsTol: The absolute tolerance error for ode solver.
	 * @param [in] RelTol: The relative tolerance error for ode solver.
	 * @param [in] maxNumSteps: The maximum number of integration points per a second
	 * for ode solver.
	 * @param [in] concatOutput: Whether to concatenate the output to the input trajectories
	 * or override (default).
	 */
	void integrate(
			const state_vector_t& initialState,
			typename scalar_array_t::const_iterator beginTimeItr,
			typename scalar_array_t::const_iterator endTimeItr,
			state_vector_array_t& stateTrajectory,
			scalar_t dtInitial = 0.01,
			scalar_t AbsTol = 1e-9,
			scalar_t RelTol = 1e-6,
			int maxNumSteps = std::numeric_limits<int>::max(),
			bool concatOutput = false) final;

private:

	/**
	 * Setup System
	 */
	void setupSystem();

	/**
	 * Initializes the integrator.
	 *
	 * @param [in] initialState
	 * @param [in] t
	 * @param [in] dt
	 */
	void initialize(
			state_vector_t& initialState,
			scalar_t& t,
			scalar_t dt);

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
			scalar_t RelTol);

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
			scalar_t RelTol);

	/**
	 * Integrate times specialized function
	 * @tparam S: stepper type.
	 * @param [in] initialState: Initial state.
	 * @param [in] beginTimeItr: The iterator to the beginning of the time stamp trajectory.
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
			scalar_t RelTol);

	/**
	 * Integrate times specialized function
	 * @tparam S: stepper type.
	 * @param [in] initialState: Initial state.
	 * @param [in] beginTimeItr: The iterator to the beginning of the time stamp trajectory.
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
			scalar_t RelTol);

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
		initializeStepper(
				state_vector_t& initialState,
				scalar_t& t,
				scalar_t dt);

	/**
	 * Functionality to reset stepper. If we integrate with some other method, e.g.
	 * adams_bashforth, we need to reset the stepper, hence specialize with initialize call
	 *
	 * @tparam S
	 * @param [in] initialState
	 * @param [in] t: Time.
	 * @param [in] dt: Time step.
	 * @return
	 */
	template <typename S = Stepper>
	typename std::enable_if<!(std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value), void>::type
		initializeStepper(
				state_vector_t& initialState,
				scalar_t& t,
				scalar_t dt);

	/********
	 * Variables
	 ********/
	std::function<void (
			const Eigen::Matrix<scalar_t, STATE_DIM, 1>&,
			Eigen::Matrix<scalar_t, STATE_DIM, 1>&,
			scalar_t)> systemFunction_;

	Stepper stepper_;

#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 60)
	std::unique_ptr<boost::numeric::odeint::max_step_checker> maxStepCheckerPtr_;
#endif

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

#include "implementation/Integrator.h"

#endif /* INTEGRATOR_OCS2_H_ */
