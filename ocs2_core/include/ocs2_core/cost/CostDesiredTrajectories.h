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
#include <iomanip>
#include <iostream>
#include <vector>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/misc/LinearInterpolation.h"

namespace ocs2 {

/**
 * This class is an interface class for the cost desired trajectories.
 */
class CostDesiredTrajectories {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using DIMENSIONS = Dimensions<0, 0>;
  using scalar_t = DIMENSIONS::scalar_t;
  using scalar_array_t = DIMENSIONS::scalar_array_t;
  using dynamic_vector_t = DIMENSIONS::dynamic_vector_t;
  using dynamic_vector_array_t = DIMENSIONS::dynamic_vector_array_t;

  explicit CostDesiredTrajectories(const scalar_array_t& desiredTimeTrajectory = scalar_array_t(),
                                   const dynamic_vector_array_t& desiredStateTrajectory = dynamic_vector_array_t(),
                                   const dynamic_vector_array_t& desiredInputTrajectory = dynamic_vector_array_t())
      : desiredTimeTrajectory_(desiredTimeTrajectory),
        desiredStateTrajectory_(desiredStateTrajectory),
        desiredInputTrajectory_(desiredInputTrajectory) {}

  explicit CostDesiredTrajectories(const size_t& trajectorySize)
      : desiredTimeTrajectory_(trajectorySize), desiredStateTrajectory_(trajectorySize), desiredInputTrajectory_(trajectorySize) {}

  ~CostDesiredTrajectories() = default;

  bool empty() const { return desiredTimeTrajectory_.empty(); }

  void clear() {
    desiredTimeTrajectory_.clear();
    desiredStateTrajectory_.clear();
    desiredInputTrajectory_.clear();
  }

  void swap(CostDesiredTrajectories& other) {
    desiredTimeTrajectory_.swap(other.desiredTimeTrajectory_);
    desiredStateTrajectory_.swap(other.desiredStateTrajectory_);
    desiredInputTrajectory_.swap(other.desiredInputTrajectory_);
  }

  bool operator==(const CostDesiredTrajectories& other) {
    return this->desiredTimeTrajectory() == other.desiredTimeTrajectory() &&
           this->desiredStateTrajectory() == other.desiredStateTrajectory() &&
           this->desiredInputTrajectory() == other.desiredInputTrajectory();
  }

  bool operator!=(const CostDesiredTrajectories& other) { return !(*this == other); }

  scalar_array_t& desiredTimeTrajectory() { return desiredTimeTrajectory_; }
  const scalar_array_t& desiredTimeTrajectory() const { return desiredTimeTrajectory_; }

  dynamic_vector_array_t& desiredStateTrajectory() { return desiredStateTrajectory_; }
  const dynamic_vector_array_t& desiredStateTrajectory() const { return desiredStateTrajectory_; }

  dynamic_vector_array_t& desiredInputTrajectory() { return desiredInputTrajectory_; }
  const dynamic_vector_array_t& desiredInputTrajectory() const { return desiredInputTrajectory_; }

  void getDesiredState(scalar_t time, dynamic_vector_t& desiredState) const {
    if (desiredTimeTrajectory_.empty() || desiredStateTrajectory_.empty()) {
      desiredState.setZero();
    } else {
      EigenLinearInterpolation<dynamic_vector_t>::interpolate(time, desiredState, &desiredTimeTrajectory_, &desiredStateTrajectory_);
    }
  }

  void getDesiredInput(scalar_t time, dynamic_vector_t& desiredInput) const {
    if (desiredTimeTrajectory_.empty() || desiredInputTrajectory_.empty()) {
      desiredInput.setZero();
    } else {
      EigenLinearInterpolation<dynamic_vector_t>::interpolate(time, desiredInput, &desiredTimeTrajectory_, &desiredInputTrajectory_);
    }
  }

  void display() const {
    const int dispPrecision = 4;

    size_t N = desiredTimeTrajectory_.size();
    for (size_t i = 0; i < N; i++) {
      std::cerr << "time: " << std::setprecision(dispPrecision) << desiredTimeTrajectory_[i] << ",  " << std::endl;

      // state
      std::cerr << "state: [";
      for (size_t j = 0; j < desiredStateTrajectory_[i].size(); j++) {
        std::cerr << std::setprecision(dispPrecision) << desiredStateTrajectory_[i](j) << ",  ";
      }
      if (desiredStateTrajectory_[i].size() > 0) {
        std::cerr << "\b\b]"
                  << ",  " << std::endl;
      } else {
        std::cerr << " ]"
                  << ",  " << std::endl;
      }

      // input
      std::cerr << "input: [";
      for (size_t j = 0; j < desiredInputTrajectory_[i].size(); j++) {
        std::cerr << std::setprecision(dispPrecision) << desiredInputTrajectory_[i](j) << ",  ";
      }
      if (desiredInputTrajectory_[i].size() > 0) {
        std::cerr << "\b\b]" << std::endl;
      } else {
        std::cerr << " ]" << std::endl;
      }

    }  // end of i loop
  }

 private:
  scalar_array_t desiredTimeTrajectory_;
  dynamic_vector_array_t desiredStateTrajectory_;
  dynamic_vector_array_t desiredInputTrajectory_;
};

}  // namespace ocs2
