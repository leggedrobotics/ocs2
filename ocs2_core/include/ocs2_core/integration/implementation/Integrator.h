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
Integrator<STATE_DIM, Stepper>::Integrator(const std::shared_ptr<OdeBase<STATE_DIM>>& systemPtr,
                                           const std::shared_ptr<SystemEventHandler<STATE_DIM>>& eventHandlerPtr /*= nullptr*/)

    : BASE(systemPtr, eventHandlerPtr), stepperPtr_(new Stepper), observerPtr_(new Observer<STATE_DIM>(eventHandlerPtr)) {
  // setup observer function
  observerFunction_ = [this](const state_vector_t& x, const scalar_t& t) { this->observerPtr_->observe(this->systemPtr_, x, t); };

  // setup system function
  systemFunction_ = [this](const state_vector_t& x, state_vector_t& dxdt, const scalar_t& t) {
    this->systemPtr_->computeFlowMap(t, x, dxdt);
  };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
void Integrator<STATE_DIM, Stepper>::runIntegration(const state_vector_t& initialState, const scalar_t& startTime,
                                                    const scalar_t& finalTime, scalar_t dt, state_vector_array_t& stateTrajectory,
                                                    scalar_array_t& timeTrajectory, bool concatOutput) {
  state_vector_t initialStateInternal = initialState;

  /*
   * use a temporary state for initialization, the state returned by initialize is different
   * from the real init state (already forward integrated)
   */
  state_vector_t initialStateInternal_init_temp = initialState;

  scalar_t startTime_temp = startTime;

  // reset the trajectories
  if (!concatOutput) {
    timeTrajectory.clear();
    stateTrajectory.clear();
    if (this->getModelDataTrajectoryPtr()) {
      this->getModelDataTrajectoryPtr()->clear();
    }
  }

  observerPtr_->setStateTrajectory(&stateTrajectory);
  observerPtr_->setTimeTrajectory(&timeTrajectory);
  observerPtr_->setModelDataTrajectory(this->getModelDataTrajectoryPtr());

  initialize(initialStateInternal_init_temp, startTime_temp, dt);

  boost::numeric::odeint::integrate_const(*stepperPtr_, systemFunction_, initialStateInternal, startTime, finalTime + 0.1 * dt, dt,
                                          observerFunction_);
  observerPtr_->setModelDataTrajectory(nullptr);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
void Integrator<STATE_DIM, Stepper>::runIntegration(const state_vector_t& initialState, const scalar_t& startTime,
                                                    const scalar_t& finalTime, state_vector_array_t& stateTrajectory,
                                                    scalar_array_t& timeTrajectory, scalar_t dtInitial, scalar_t AbsTol, scalar_t RelTol,
                                                    int maxNumSteps, bool concatOutput) {
  state_vector_t internalStartState = initialState;

  if (this->eventHandlerPtr_ && maxNumSteps < std::numeric_limits<int>::max()) {
    this->eventHandlerPtr_->setMaxNumSteps(maxNumSteps);
  }

  // reset the trajectories
  if (!concatOutput) {
    timeTrajectory.clear();
    stateTrajectory.clear();
    if (this->getModelDataTrajectoryPtr()) {
      this->getModelDataTrajectoryPtr()->clear();
    }
  }

  observerPtr_->setStateTrajectory(&stateTrajectory);
  observerPtr_->setTimeTrajectory(&timeTrajectory);
  observerPtr_->setModelDataTrajectory(this->getModelDataTrajectoryPtr());

  integrate_adaptive_specialized<Stepper>(internalStartState, startTime, finalTime, dtInitial, AbsTol, RelTol);
  observerPtr_->setModelDataTrajectory(nullptr);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
void Integrator<STATE_DIM, Stepper>::runIntegration(const state_vector_t& initialState,
                                                    typename scalar_array_t::const_iterator beginTimeItr,
                                                    typename scalar_array_t::const_iterator endTimeItr,
                                                    state_vector_array_t& stateTrajectory, scalar_t dtInitial, scalar_t AbsTol,
                                                    scalar_t RelTol, int maxNumSteps, bool concatOutput) {
  state_vector_t internalStartState = initialState;

  if (this->eventHandlerPtr_ && maxNumSteps < std::numeric_limits<int>::max()) {
    this->eventHandlerPtr_->setMaxNumSteps(maxNumSteps);
  }

#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 60)
  if (!maxStepCheckerPtr_ || !concatOutput) {
    maxStepCheckerPtr_.reset(new boost::numeric::odeint::max_step_checker(maxNumSteps));
  }
#endif

  // reset the trajectories
  if (!concatOutput) {
    stateTrajectory.clear();
    if (this->getModelDataTrajectoryPtr()) {
      this->getModelDataTrajectoryPtr()->clear();
    }
  }

  observerPtr_->setStateTrajectory(&stateTrajectory);
  observerPtr_->setModelDataTrajectory(this->getModelDataTrajectoryPtr());

  integrate_times_specialized<Stepper>(internalStartState, beginTimeItr, endTimeItr, dtInitial, AbsTol, RelTol);
  observerPtr_->setModelDataTrajectory(nullptr);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
void Integrator<STATE_DIM, Stepper>::initialize(state_vector_t& initialState, scalar_t& t, scalar_t dt) {
  //	initializeStepper(initialState, t, dt);	// TODO
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
Integrator<STATE_DIM, Stepper>::integrate_adaptive_specialized(state_vector_t& initialState, const scalar_t& startTime,
                                                               const scalar_t& finalTime, scalar_t dtInitial, scalar_t AbsTol,
                                                               scalar_t RelTol) {
  boost::numeric::odeint::integrate_adaptive(boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol), systemFunction_, initialState,
                                             startTime, finalTime, dtInitial, observerFunction_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<!std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
Integrator<STATE_DIM, Stepper>::integrate_adaptive_specialized(state_vector_t& initialState, const scalar_t& startTime,
                                                               const scalar_t& finalTime, scalar_t dtInitial, scalar_t AbsTol,
                                                               scalar_t RelTol) {
  boost::numeric::odeint::integrate_adaptive(*stepperPtr_, systemFunction_, initialState, startTime, finalTime, dtInitial,
                                             observerFunction_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
Integrator<STATE_DIM, Stepper>::integrate_times_specialized(state_vector_t& initialState,
                                                            typename scalar_array_t::const_iterator beginTimeItr,
                                                            typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial,
                                                            scalar_t AbsTol, scalar_t RelTol) {
#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 60)
  boost::numeric::odeint::integrate_times(boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol), systemFunction_, initialState,
                                          beginTimeItr, endTimeItr, dtInitial, observerFunction_, *maxStepCheckerPtr_);

#else
  boost::numeric::odeint::integrate_times(boost::numeric::odeint::make_controlled<S>(AbsTol, RelTol), systemFunction_, initialState,
                                          beginTimeItr, endTimeItr, dtInitial, observerFunction_);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<!std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
Integrator<STATE_DIM, Stepper>::integrate_times_specialized(state_vector_t& initialState,
                                                            typename scalar_array_t::const_iterator beginTimeItr,
                                                            typename scalar_array_t::const_iterator endTimeItr, scalar_t dtInitial,
                                                            scalar_t AbsTol, scalar_t RelTol) {
#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 60)
  boost::numeric::odeint::integrate_times(*stepperPtr_, systemFunction_, initialState, beginTimeItr, endTimeItr, dtInitial,
                                          observerFunction_, *maxStepCheckerPtr_);

#else
  boost::numeric::odeint::integrate_times(*stepperPtr_, systemFunction_, initialState, beginTimeItr, endTimeItr, dtInitial,
                                          observerFunction_);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value, void>::type
Integrator<STATE_DIM, Stepper>::initializeStepper(state_vector_t& initialState, scalar_t& t, scalar_t dt) {
  /**do nothing, runge_kutta_5_t does not have a init method */
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <int STATE_DIM, class Stepper>
template <typename S>
typename std::enable_if<!(std::is_same<S, runge_kutta_dopri5_t<STATE_DIM>>::value), void>::type
Integrator<STATE_DIM, Stepper>::initializeStepper(state_vector_t& initialState, scalar_t& t, scalar_t dt) {
  stepperPtr_->initialize(runge_kutta_dopri5_t<STATE_DIM>(), systemFunction_, initialState, t, dt);
}

}  // namespace ocs2
