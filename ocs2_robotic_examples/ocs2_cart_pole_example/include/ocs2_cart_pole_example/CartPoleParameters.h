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
#include <string>
#include <vector>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace ocs2 {
namespace cartpole {

template <typename SCALAR_T>
class CartPoleParameters {
 public:
  /**
   * Constructor.
   */
  explicit CartPoleParameters(SCALAR_T cartMass = 1.0, SCALAR_T poleMass = 1.0, SCALAR_T poleLength = 1.0, SCALAR_T gravity = 9.8)
      : cartMass_(cartMass), poleMass_(poleMass), poleLength_(poleLength), gravity_(gravity) {
    computeInertiaTerms();
  }

  /**
   * Default destructor.
   */
  ~CartPoleParameters() = default;

  /**
   * Displays the Cart-Pole's parameters.
   */
  inline void display() {
    std::cerr << "Cart-pole parameters: " << std::endl;
    std::cerr << "cartMass:   " << cartMass_ << std::endl;
    std::cerr << "poleMass:   " << poleMass_ << std::endl;
    std::cerr << "poleLength: " << poleLength_ << std::endl;
    std::cerr << "poleMoi:    " << poleMoi_ << std::endl;
    std::cerr << "gravity:    " << gravity_ << std::endl;
  }

  /**
   * Loads the Cart-Pole's parameters.
   */
  inline void loadSettings(const std::string& filename, bool verbose = true) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    if (verbose) {
      std::cerr << "\n #### Cart-pole Parameters:" << std::endl;
    }
    if (verbose) {
      std::cerr << " #### =========================================" << std::endl;
    }

    try {
      cartMass_ = pt.get<SCALAR_T>("CartPoleParameters.cartMass");
      if (verbose) {
        std::cerr << " #### cartMass ......... " << cartMass_ << std::endl;
      }
    } catch (const std::exception& e) {
      if (verbose) {
        std::cerr << " #### cartMass ......... " << cartMass_ << "\t(default)" << std::endl;
      }
    }

    try {
      poleMass_ = pt.get<SCALAR_T>("CartPoleParameters.poleMass");
      if (verbose) {
        std::cerr << " #### poleMass ......... " << poleMass_ << std::endl;
      }
    } catch (const std::exception& e) {
      if (verbose) {
        std::cerr << " #### poleMass ......... " << poleMass_ << "\t(default)" << std::endl;
      }
    }

    try {
      poleLength_ = pt.get<SCALAR_T>("CartPoleParameters.poleLength");
      if (verbose) {
        std::cerr << " #### poleLength ....... " << poleLength_ << std::endl;
      }
    } catch (const std::exception& e) {
      if (verbose) {
        std::cerr << " #### poleLength ....... " << poleLength_ << "\t(default)" << std::endl;
      }
    }

    try {
      gravity_ = pt.get<SCALAR_T>("CartPoleParameters.gravity");
      if (verbose) {
        std::cerr << " #### gravity .......... " << gravity_ << std::endl;
      }
    } catch (const std::exception& e) {
      if (verbose) {
        std::cerr << " #### gravity .......... " << gravity_ << "\t(default)" << std::endl;
      }
    }

    computeInertiaTerms();
  }

 public:
  // For safety, these parameters cannot be modified
  SCALAR_T cartMass_;        // [kg]
  SCALAR_T poleMass_;        // [kg]
  SCALAR_T poleLength_;      // [m]
  SCALAR_T poleHalfLength_;  // [m]
  SCALAR_T poleMoi_;         // [kg*m^2]
  SCALAR_T poleSteinerMoi_;  // [kg*m^2]
  SCALAR_T gravity_;         // [m/s^2]

 private:
  void computeInertiaTerms() {
    poleHalfLength_ = poleLength_ / 2.0;
    poleMoi_ = 1.0 / 12.0 * poleMass_ * (poleLength_ * poleLength_);
    poleSteinerMoi_ = poleMoi_ + poleMass_ * (poleHalfLength_ * poleHalfLength_);
  }
};

}  // namespace cartpole
}  // namespace ocs2
