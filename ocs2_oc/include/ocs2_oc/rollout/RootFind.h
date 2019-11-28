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

#include <iostream>
#include <utility>
#include <vector>

#include <ocs2_core/Dimensions.h>

namespace ocs2 {

/**
 * This class is used in StateTriggerdRollout to find a new query point,
 * based on a previously known interval, which is the best guess to find
 * the root of the guardSurface.
 *
 * It assumes that on initialization an interval is known on which the
 * guard function changes sign, and that within this interval a single root
 * is located.
 *
 * The implemented algorithms are:
 * Regular Regula Falsi:
 * form: [Improved Algorithms of Illinois-Type for the Numerical Solution of Non-
 * 	      Linear Equations by J.A. Ford]
 * Illinois method:
 * from: [A Modified Regula Falsi Method for Computing the Root of an Equation
 *        by M. Dowell and P. Jarrat]
 * Pegasus method:
 * from: [The "Pegasus" Method for Computing the Root of an Equation
 *        by M. Dowell and P. Jarrat]
 * Anderson & Bjorck Method (default):
 * from: [A New High Order Method of Regula Falsi Type for Computing a Root of an Equation
 *        by N.Anderson and A. Bjorck]
 *
 *
 */
class RootFinder {
 public:
  using scalar_t = Dimensions<0, 0>::scalar_t;
  using interval_t = std::pair<scalar_t, scalar_t>;

  /**
   * Default Constructor.
   */
  RootFinder(int rootFindingAlgorithm): rootFindingAlgorithm_(rootFindingAlgorithm){}

  /**
   * Default destructor.
   */
  ~RootFinder() = default;

  /**
   * Set the initial bracket when the RootFinding method is initialized.
   * Normally done when first zero crossing is detected
   *
   * @param [in] timeInt: Pair of two time moments
   * @param [in] guardInt: Pair of two function values at time_int times, should have opposite sign
   */
  void setInitBracket(const interval_t& timeInt, const interval_t& guardInt) {
    if (guardInt.first * guardInt.second > 0) {
      throw std::runtime_error("Bracket function values should have opposite sign");
    }

    timeInt_ = timeInt;
    guardInt_ = guardInt;
  }

  /**
   * Set the initial bracket when the RootFinding method is initialized.
   * Normally done when first zero crossing is detected.
   *
   * @param [in] t0: First time of bracketing interval.
   * @param [in] t1: Second time of bracketing interval.
   * @param [in] f0: Function value corresponding to t0.
   * @param [in] f1: Function value corresponding to t1, of opposite sign to f0.
   */
  void setInitBracket(const scalar_t t0, const scalar_t t1, const scalar_t f0, const scalar_t f1) {
	  setInitBracket(std::make_pair(t0,t1),std::make_pair(f0,f1));
  }

  /**
   * Update Current bracket, based on based on sign of the new query point
   *
   * @param [in] query: Time moment of last query point
   * @param [in] fQuery: Function evaluation of last query time
   */
  void updateBracket(const scalar_t& query, const scalar_t& fQuery) {
    if (fQuery * guardInt_.first < 0) {
      guardInt_.second = guardInt_.first;
      timeInt_.second = timeInt_.first;

      guardInt_.first = fQuery;
      timeInt_.first = query;

    } else {
    	scalar_t gamma;
    	if (rootFindingAlgorithm_ == 0)// Anderson & Bjorck method
    	{
    		gamma = 1 - (fQuery / guardInt_.first);
    		if (gamma < 0) {
    		   gamma = 0.5;
    		}
    	}
    	else if(rootFindingAlgorithm_ == 1)// Pegasus Method
    	{
    		gamma = guardInt_.first/(guardInt_.first + fQuery);
    	}
    	else if(rootFindingAlgorithm_ == 2)// Illinois method
    	{
    		gamma = 0.5;
    	}
    	else	// Regular Regula Falsi
    	{
    		gamma = 1;
    	}

      guardInt_.first = fQuery;
      timeInt_.first = query;

      guardInt_.second *= gamma;
    }
  }

  /**
   * Use (adapted-) regula falsi method to obtain a new query point
   *
   * @param [out] query: Time moment of new query point
   *
   */
  void getNewQuery(double& query) {
    scalar_t fa = guardInt_.first;
    scalar_t ta = timeInt_.first;

    scalar_t fb = guardInt_.second;
    scalar_t tb = timeInt_.second;

    query = (ta * fb - tb * fa) / (fb - fa);
  }

  /**
   * Display relevant bracketing information
   *
   */
  void display() {
    std::cerr << "Root Finding Information" << std::endl;
    std::cerr << "Time Bracket: [" << timeInt_.first << ";" << timeInt_.second << "]" << std::endl;
    std::cerr << "Value Bracket: [" << guardInt_.first << ";" << guardInt_.second << "]" << std::endl;
  }

 private:
  interval_t timeInt_;
  interval_t guardInt_;
  size_t rootFindingAlgorithm_;
};

}  // namespace ocs2
