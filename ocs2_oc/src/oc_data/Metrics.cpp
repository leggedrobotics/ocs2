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

#include "ocs2_oc/oc_data/Metrics.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void swap(Metrics& lhs, Metrics& rhs) {
  // Cost
  std::swap(lhs.cost, rhs.cost);

  // Equality constraints
  lhs.stateEqConstraint.swap(rhs.stateEqConstraint);
  lhs.stateInputEqConstraint.swap(rhs.stateInputEqConstraint);

  // Lagrangians
  //  lhs.stateEqLagrangian.swap(rhs.stateEqLagrangian);
  //  lhs.stateIneqLagrangian.swap(rhs.stateIneqLagrangian);
  //  lhs.stateInputEqLagrangian.swap(rhs.stateInputEqLagrangian);
  //  lhs.stateInputIneqLagrangian.swap(rhs.stateInputIneqLagrangian);

  std::swap(lhs.stateEqLagrangian, rhs.stateEqLagrangian);
  std::swap(lhs.stateIneqLagrangian, rhs.stateIneqLagrangian);
  std::swap(lhs.stateInputEqLagrangian, rhs.stateInputEqLagrangian);
  std::swap(lhs.stateInputIneqLagrangian, rhs.stateInputIneqLagrangian);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void clear(Metrics& m) {
  // Cost
  m.cost = 0.0;

  // Equality constraints
  m.stateEqConstraint = vector_t();
  m.stateInputEqConstraint = vector_t();

  // Lagrangians
  //  m.stateEqLagrangian.clear();
  //  m.stateIneqLagrangian.clear();
  //  m.stateInputEqLagrangian.clear();
  //  m.stateInputIneqLagrangian.clear();

  m.stateEqLagrangian = 0.0;
  m.stateIneqLagrangian = 0.0;
  m.stateInputEqLagrangian = 0.0;
  m.stateInputIneqLagrangian = 0.0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void swap(MetricsCollection& lhs, MetricsCollection& rhs) {
  swap(lhs.final, rhs.final);
  lhs.preJumps.swap(rhs.preJumps);
  lhs.intermediates.swap(rhs.intermediates);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void clear(MetricsCollection& m) {
  clear(m.final);
  m.preJumps.clear();
  m.intermediates.clear();
}

}  // namespace ocs2
