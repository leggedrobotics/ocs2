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

#include <Eigen/Dense>

#include <ocs2_core/Types.h>

using scalar_t = ocs2::scalar_t;
using vector_t = ocs2::vector_t;
using matrix_t = ocs2::matrix_t;
using scalar_array_t = ocs2::scalar_array_t;
using vector_array_t = ocs2::vector_array_t;

/*
 * 	This class constructs, contains a section of the reference, which is contained in
 * 	OverallReference and used in BouncingMass SLQ test.
 * 	The reference is based on the reference presented in
 * 	"On Optimal Trajectory Tracking for Mechanical Systems with Unilateral Constraints" by M. Rijnen
 */
class Reference {
 public:
  /*
   * Constructor
   *
   * @param [in] t0: Time at which the first waypoint of the reference is defined
   * @param [in] t1: Time at which the second waypoint of the reference is defined
   * @param [in] p0: First waypoint of the reference [position,velocity,acceleration]
   * @param [in] p1: Second waypoint of the reference
   */
  Reference(scalar_t t0, scalar_t t1, const vector_t& p0, const vector_t& p1);

  /*
   * Obtain reference input at the current time
   *
   * @param [in] time: current time
   * @return current reference input
   */
  vector_t getInput(const scalar_t time) const;

  /*
   * Obtain reference state at current time
   *
   * @param [in] time: current time
   * @return current reference state
   */
  vector_t getState(const scalar_t time) const;

  /*
   * Extend the reference by integrating the input signal of the next
   * and previous sections
   *
   * @param [in] delta: extension time
   * @param [in] refPre: pointer to previous section of the reference
   * @param [in] refPost: pointer to the next section of the reference
   */
  void extendref(scalar_t delta, Reference* refPre, Reference* refPost);

  /* Display the reference */
  void display();

  scalar_t t0_;
  scalar_t t1_;

 private:
  /*
   * 	Find the polynomial coefficients corresponding to the posed constraints
   * 	on position, velocity and accleration at the posed times
   *
   * 	@param [in]	t0:	Time moment of first waypoint
   * 	@param [in] t1: time moment of second waypoint
   * 	@param [in] p0: first waypoint
   * 	@param [in] p1: second waypoint
   */
  Eigen::Matrix<scalar_t, 6, 1> Create5thOrdPol(scalar_t t0, scalar_t t1, const vector_t& p0, const vector_t& p1) const;

  /*
   * 	Find the derivative of the polynomial described by a
   * 	set of coefficients
   *
   * 	@param [in]	pol: coefficients describing the fifth order polynomial
   */
  Eigen::Matrix<scalar_t, 6, 1> polyder(Eigen::Matrix<scalar_t, 6, 1> pol);

  /*
   * 	Interpolate the extended reference signals to find the reference
   * 	corresponding to the current time
   *
   * 	@param [in]	time:	current time
   * 	@param [out] x: current state
   */
  void interpolate_ext(scalar_t time, vector_t& x) const;

  Eigen::Matrix<scalar_t, 6, 1> polU_;
  Eigen::Matrix<scalar_t, 6, 1> polX_;
  Eigen::Matrix<scalar_t, 6, 1> polV_;

  vector_array_t xPre_;
  scalar_array_t tPre_;
  vector_array_t xPost_;
  scalar_array_t tPost_;
};
