/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <iomanip>
#include <iostream>

#include <ocs2_core/cost/CostDesiredTrajectories.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
CostDesiredTrajectories::CostDesiredTrajectories(scalar_array_t desiredTimeTrajectory, vector_array_t desiredStateTrajectory,
                                                 vector_array_t desiredInputTrajectory)
    : desiredTimeTrajectory_(std::move(desiredTimeTrajectory)),
      desiredStateTrajectory_(std::move(desiredStateTrajectory)),
      desiredInputTrajectory_(std::move(desiredInputTrajectory)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
CostDesiredTrajectories::CostDesiredTrajectories(size_t trajectorySize)
    : desiredTimeTrajectory_(trajectorySize), desiredStateTrajectory_(trajectorySize), desiredInputTrajectory_(trajectorySize) {}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
bool CostDesiredTrajectories::empty() const {
  return desiredTimeTrajectory_.empty();
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void CostDesiredTrajectories::clear() {
  desiredTimeTrajectory_.clear();
  desiredStateTrajectory_.clear();
  desiredInputTrajectory_.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void CostDesiredTrajectories::swap(CostDesiredTrajectories& other) {
  desiredTimeTrajectory_.swap(other.desiredTimeTrajectory_);
  desiredStateTrajectory_.swap(other.desiredStateTrajectory_);
  desiredInputTrajectory_.swap(other.desiredInputTrajectory_);
  // std::swap()
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
bool CostDesiredTrajectories::operator==(const CostDesiredTrajectories& other) {
  return this->desiredTimeTrajectory() == other.desiredTimeTrajectory() &&
         this->desiredStateTrajectory() == other.desiredStateTrajectory() &&
         this->desiredInputTrajectory() == other.desiredInputTrajectory();
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
vector_t CostDesiredTrajectories::getDesiredState(scalar_t time) const {
  vector_t desiredState;
  if (desiredTimeTrajectory_.empty() || desiredStateTrajectory_.empty()) {
    // TODO(mspieler): error handling
    throw std::runtime_error("CostDesiredTrajectories is empty.");
  } else {
    LinearInterpolation::interpolate(time, desiredState, &desiredTimeTrajectory_, &desiredStateTrajectory_);
  }
  return desiredState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
vector_t CostDesiredTrajectories::getDesiredInput(scalar_t time) const {
  vector_t desiredInput;
  if (desiredTimeTrajectory_.empty() || desiredInputTrajectory_.empty()) {
    // TODO(mspieler): error handling
    throw std::runtime_error("CostDesiredTrajectories is empty.");
  } else {
    LinearInterpolation::interpolate(time, desiredInput, &desiredTimeTrajectory_, &desiredInputTrajectory_);
  }
  return desiredInput;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
std::ostream& operator<<(std::ostream& out, const CostDesiredTrajectories& traj) {
  const int dispPrecision = 4;

  for (size_t i = 0; i < traj.desiredTimeTrajectory_.size(); i++) {
    out << "time: " << std::setprecision(dispPrecision) << traj.desiredTimeTrajectory_[i] << ",\n";

    // state
    out << "state: [";
    for (size_t j = 0; j < traj.desiredStateTrajectory_[i].size(); j++) {
      out << std::setprecision(dispPrecision) << traj.desiredStateTrajectory_[i](j) << ",  ";
    }
    if (traj.desiredStateTrajectory_[i].size() > 0) {
      out << "\b\b]"
          << ",\n";
    } else {
      out << " ]"
          << ",\n";
    }

    // input
    out << "input: [";
    for (size_t j = 0; j < traj.desiredInputTrajectory_[i].size(); j++) {
      out << std::setprecision(dispPrecision) << traj.desiredInputTrajectory_[i](j) << ",  ";
    }
    if (traj.desiredInputTrajectory_[i].size() > 0) {
      out << "\b\b]\n";
    } else {
      out << " ]\n";
    }

  }  // end of i loop
  return out;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void CostDesiredTrajectories::display() const {
  std::cerr << *this;
}

}  // namespace ocs2
