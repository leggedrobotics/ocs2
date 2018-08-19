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

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
Integrator<STATE_DIM, Stepper>::Integrator(
		const std::shared_ptr<ODE_Base<STATE_DIM> >& systemPtr,
		const std::shared_ptr<SystemEventHandler<STATE_DIM> >& eventHandlerPtr /*= nullptr*/)

	: BASE(systemPtr, eventHandlerPtr)
{
	setupSystem();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
void Integrator<STATE_DIM, Stepper>::integrate(
		const state_vector_t& initialState,
		const scalar_t& startTime,
		const scalar_t& finalTime,
		scalar_t dt,
		state_vector_array_t& stateTrajectory,
		scalar_array_t& timeTrajectory,
		bool concatOutput /*= false*/) {

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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
void Integrator<STATE_DIM, Stepper>::integrate(
		const state_vector_t& initialState,
		const scalar_t& startTime,
		const scalar_t& finalTime,
		state_vector_array_t& stateTrajectory,
		scalar_array_t& timeTrajectory,
		scalar_t dtInitial /*= 0.01*/,
		scalar_t AbsTol /*= 1e-6*/,
		scalar_t RelTol /*= 1e-3*/,
		size_t maxNumSteps /*= std::numeric_limits<size_t>::max()*/,
		bool concatOutput /*= false*/)  {

	state_vector_t internalStartState = initialState;

	if (BASE::eventHandlerPtr_ && maxNumSteps<std::numeric_limits<size_t>::max())
		BASE::eventHandlerPtr_->setMaxNumSteps(maxNumSteps);

#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)
	if (!maxStepCheckerPtr_ || concatOutput==false)
		maxStepCheckerPtr_.reset(new boost::numeric::odeint::max_step_checker(maxNumSteps));
#endif

	// reset the trajectories
	if (concatOutput==false) {
		timeTrajectory.clear();
		stateTrajectory.clear();
	}

	BASE::setOutputTrajectoryPtrToObserver(&stateTrajectory, &timeTrajectory);

	integrate_adaptive_specialized<Stepper>(
			internalStartState, startTime, finalTime, dtInitial, AbsTol, RelTol);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
void Integrator<STATE_DIM, Stepper>::integrate(
		const state_vector_t& initialState,
		typename scalar_array_t::const_iterator beginTimeItr,
		typename scalar_array_t::const_iterator endTimeItr,
		state_vector_array_t& stateTrajectory,
		scalar_t dtInitial /*= 0.01*/,
		scalar_t AbsTol /*= 1e-9*/,
		scalar_t RelTol /*= 1e-6*/,
		size_t maxNumSteps /*= std::numeric_limits<size_t>::max()*/,
		bool concatOutput /*= false*/)  {

	state_vector_t internalStartState = initialState;

	if (BASE::eventHandlerPtr_ && maxNumSteps<std::numeric_limits<size_t>::max())
		BASE::eventHandlerPtr_->setMaxNumSteps(maxNumSteps);

#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)
	if (!maxStepCheckerPtr_ || concatOutput==false)
		maxStepCheckerPtr_.reset(new boost::numeric::odeint::max_step_checker(maxNumSteps));
#endif

	// reset the trajectories
	if (concatOutput==false) {
		stateTrajectory.clear();
	}

	BASE::setOutputTrajectoryPtrToObserver(&stateTrajectory);

	integrate_times_specialized<Stepper>(
			internalStartState, beginTimeItr, endTimeItr, dtInitial, AbsTol, RelTol);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
void Integrator<STATE_DIM, Stepper>::setupSystem() {

	systemFunction_ = [this](
			const Eigen::Matrix<scalar_t, STATE_DIM, 1>& x,
			Eigen::Matrix<scalar_t, STATE_DIM, 1>& dxdt,
			scalar_t t) {
		const state_vector_t& xState(static_cast<const state_vector_t&>(x));
		state_vector_t& dxdtState(static_cast<state_vector_t&>(dxdt));
		this->systemPtr_->computeFlowMap(t, xState, dxdtState);
	};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
void Integrator<STATE_DIM, Stepper>::initialize(
		state_vector_t& initialState,
		scalar_t& t,
		scalar_t dt) {

//	initializeStepper(initialState, t, dt);	// TODO
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
	Integrator<STATE_DIM, Stepper>::integrate_adaptive_specialized(
		state_vector_t& initialState,
		const scalar_t& startTime,
		const scalar_t& finalTime,
		scalar_t dtInitial,
		scalar_t AbsTol,
		scalar_t RelTol) {

#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)
	boost::numeric::odeint::integrate_adaptive(
			boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol),
			systemFunction_,
			initialState,
			startTime,
			finalTime,
			dtInitial,
			BASE::observer_.observeWrap,
			*maxStepCheckerPtr_);

#else
	boost::numeric::odeint::integrate_adaptive(
			boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol),
			systemFunction_,
			initialState,
			startTime,
			finalTime,
			dtInitial,
			BASE::observer_.observeWrap);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<!std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
	Integrator<STATE_DIM, Stepper>::integrate_adaptive_specialized(
		state_vector_t& initialState,
		const scalar_t& startTime,
		const scalar_t& finalTime,
		scalar_t dtInitial,
		scalar_t AbsTol,
		scalar_t RelTol) {
#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)
	boost::numeric::odeint::integrate_adaptive(
			stepper_,
			systemFunction_,
			initialState,
			startTime,
			finalTime,
			dtInitial,
			BASE::observer_.observeWrap,
			*maxStepCheckerPtr_);

#else
	boost::numeric::odeint::integrate_adaptive(
			stepper_,
			systemFunction_,
			initialState,
			startTime,
			finalTime,
			dtInitial,
			BASE::observer_.observeWrap);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
	Integrator<STATE_DIM, Stepper>::integrate_times_specialized(
		state_vector_t& initialState,
		typename scalar_array_t::const_iterator beginTimeItr,
		typename scalar_array_t::const_iterator endTimeItr,
		scalar_t dtInitial,
		scalar_t AbsTol,
		scalar_t RelTol){

	boost::numeric::odeint::integrate_times(
			boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol),
			systemFunction_,
			initialState,
			beginTimeItr,
			endTimeItr,
			dtInitial,
			BASE::observer_.observeWrap);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<!std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
	Integrator<STATE_DIM, Stepper>::integrate_times_specialized(
		state_vector_t& initialState,
		typename scalar_array_t::const_iterator beginTimeItr,
		typename scalar_array_t::const_iterator endTimeItr,
		scalar_t dtInitial,
		scalar_t AbsTol,
		scalar_t RelTol){

	boost::numeric::odeint::integrate_times(
			stepper_,
			systemFunction_,
			initialState,
			beginTimeItr,
			endTimeItr,
			dtInitial,
			BASE::observer_.observeWrap);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
	Integrator<STATE_DIM, Stepper>::initializeStepper(
			state_vector_t& initialState,
			scalar_t& t,
			scalar_t dt) {

	/**do nothing, runge_kutta_5_t does not have a init method */
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<!(std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value), void>::type
	Integrator<STATE_DIM, Stepper>::initializeStepper(
			state_vector_t& initialState,
			scalar_t& t,
			scalar_t dt) {

	stepper_.initialize(runge_kutta_dopri5_t<STATE_DIM>(), systemFunction_, initialState, t, dt);
}

} // namespace ocs2

