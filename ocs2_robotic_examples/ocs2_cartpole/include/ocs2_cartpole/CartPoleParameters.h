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

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace cartpole {

struct CartPoleParameters {
  /** Constructor */
  CartPoleParameters() { computeInertiaTerms(); }

  void display() {
    std::cerr << "Cart-pole parameters: "
              << "\n";
    std::cerr << "cartMass:   " << cartMass_ << "\n";
    std::cerr << "poleMass:   " << poleMass_ << "\n";
    std::cerr << "poleLength: " << poleLength_ << "\n";
    std::cerr << "poleMoi:    " << poleMoi_ << "\n";
    std::cerr << "maxInput:   " << maxInput_ << "\n";
    std::cerr << "gravity:    " << gravity_ << "\n";
  }

  /** Loads the Cart-Pole's parameters. */
  void loadSettings(const std::string& filename, const std::string& fieldName, bool verbose = true) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);
    if (verbose) {
      std::cerr << "\n #### Cart-pole Parameters:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, cartMass_, fieldName + ".cartMass", verbose);
    loadData::loadPtreeValue(pt, poleMass_, fieldName + ".poleMass", verbose);
    loadData::loadPtreeValue(pt, poleLength_, fieldName + ".poleLength", verbose);
    loadData::loadPtreeValue(pt, maxInput_, fieldName + ".maxInput", verbose);
    loadData::loadPtreeValue(pt, gravity_, fieldName + ".gravity", verbose);
    computeInertiaTerms();
    if (verbose) {
      std::cerr << " #### =============================================================================\n" << std::endl;
    }
  }

  scalar_t cartMass_ = 1.0;       // [kg]
  scalar_t poleMass_ = 1.0;       // [kg]
  scalar_t poleLength_ = 1.0;     // [m]
  scalar_t maxInput_ = 6.0;       // [N]
  scalar_t gravity_ = 9.8;        // [m/s^2]
  scalar_t poleHalfLength_ = -1;  // [m]
  scalar_t poleMoi_ = -1;         // [kg*m^2]
  scalar_t poleSteinerMoi_ = -1;  // [kg*m^2]

 private:
  void computeInertiaTerms() {
    poleHalfLength_ = poleLength_ / 2.0;
    poleMoi_ = 1.0 / 12.0 * poleMass_ * (poleLength_ * poleLength_);
    poleSteinerMoi_ = poleMoi_ + poleMass_ * (poleHalfLength_ * poleHalfLength_);
  }
};

}  // namespace cartpole
}  // namespace ocs2
