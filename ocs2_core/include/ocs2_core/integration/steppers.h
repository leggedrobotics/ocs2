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

#pragma once

#include <boost/numeric/odeint.hpp>

namespace ocs2 {

/**
 * Euler stepper
 */
template <int STATE_DIM>
using euler_t = boost::numeric::odeint::euler<Eigen::Matrix<double, STATE_DIM, 1>, double, Eigen::Matrix<double, STATE_DIM, 1>, double,
                                              boost::numeric::odeint::vector_space_algebra>;

/**
 * Modified_Midpoint stepper
 */
template <int STATE_DIM>
using modified_midpoint_t =
    boost::numeric::odeint::modified_midpoint<Eigen::Matrix<double, STATE_DIM, 1>, double, Eigen::Matrix<double, STATE_DIM, 1>, double,
                                              boost::numeric::odeint::vector_space_algebra>;

/**
 * 4th order Runge_Kutta stepper
 */
template <int STATE_DIM>
using runge_kutta_4_t =
    boost::numeric::odeint::runge_kutta4<Eigen::Matrix<double, STATE_DIM, 1>, double, Eigen::Matrix<double, STATE_DIM, 1>, double,
                                         boost::numeric::odeint::vector_space_algebra>;

/**
 * 5th order Runge_Kutta_Dopri stepper
 */
template <int STATE_DIM>
using runge_kutta_dopri5_t =
    boost::numeric::odeint::runge_kutta_dopri5<Eigen::Matrix<double, STATE_DIM, 1>, double, Eigen::Matrix<double, STATE_DIM, 1>, double,
                                               boost::numeric::odeint::vector_space_algebra>;

/**
 * Dense_output Runge_Kutta stepper
 */
template <int STATE_DIM>
using dense_runge_kutta5_t =
    boost::numeric::odeint::dense_output_runge_kutta<boost::numeric::odeint::controlled_runge_kutta<runge_kutta_dopri5_t<STATE_DIM>>>;

/**
 * Bulirsch_Stoer stepper
 */
template <int STATE_DIM>
using bulirsch_stoer_t =
    boost::numeric::odeint::bulirsch_stoer<Eigen::Matrix<double, STATE_DIM, 1>, double, Eigen::Matrix<double, STATE_DIM, 1>, double,
                                           boost::numeric::odeint::vector_space_algebra>;

/**
 * Adams_Bashforth stepper
 */
template <int STATE_DIM, size_t STEPS>
using adams_bashforth_uncontrolled_t = boost::numeric::odeint::adams_bashforth<STEPS, Eigen::Matrix<double, STATE_DIM, 1>,  // state
                                                                               double,                               // typename value
                                                                               Eigen::Matrix<double, STATE_DIM, 1>,  // derivative
                                                                               double,                               // typename time
                                                                               boost::numeric::odeint::vector_space_algebra>;

/**
 * Adams-Bashforth-Moulton integrator (works only after boost 1.56)
 */
#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)
template <int STATE_DIM, size_t STEPS>
using adams_bashforth_moulton_uncontrolled_t =
    boost::numeric::odeint::adams_bashforth_moulton<STEPS, Eigen::Matrix<double, STATE_DIM, 1>,  // state
                                                    double,                                      // typename value
                                                    Eigen::Matrix<double, STATE_DIM, 1>,         // derivative
                                                    double,                                      // typename time
                                                    boost::numeric::odeint::vector_space_algebra>;
#endif

}  // namespace ocs2
