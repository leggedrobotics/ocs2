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

#include "ocs2_core/model_data/Metrics.h"

#include <ocs2_core/misc/Numerics.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Metrics::swap(Metrics& other) {
  // Cost
  std::swap(cost, other.cost);
  // Dynamics violation
  dynamicsViolation.swap(other.dynamicsViolation);
  // Equality constraints
  stateEqConstraint.swap(other.stateEqConstraint);
  stateInputEqConstraint.swap(other.stateInputEqConstraint);
  // Inequality constraints
  stateIneqConstraint.swap(other.stateIneqConstraint);
  stateInputIneqConstraint.swap(other.stateInputIneqConstraint);
  // Lagrangians
  stateEqLagrangian.swap(other.stateEqLagrangian);
  stateIneqLagrangian.swap(other.stateIneqLagrangian);
  stateInputEqLagrangian.swap(other.stateInputEqLagrangian);
  stateInputIneqLagrangian.swap(other.stateInputIneqLagrangian);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void Metrics::clear() {
  // Cost
  cost = 0.0;
  // Dynamics violation
  dynamicsViolation = vector_t();
  // Equality constraints
  stateEqConstraint.clear();
  stateInputEqConstraint.clear();
  // Inequality constraints
  stateIneqConstraint.clear();
  stateInputIneqConstraint.clear();
  // Lagrangians
  stateEqLagrangian.clear();
  stateIneqLagrangian.clear();
  stateInputEqLagrangian.clear();
  stateInputIneqLagrangian.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool Metrics::isApprox(const Metrics& other, scalar_t prec) const {
  return numerics::almost_eq(this->cost, other.cost, prec) && this->dynamicsViolation.isApprox(other.dynamicsViolation, prec) &&
         toVector(this->stateEqConstraint).isApprox(toVector(other.stateEqConstraint), prec) &&
         toVector(this->stateInputEqConstraint).isApprox(toVector(other.stateInputEqConstraint), prec) &&
         toVector(this->stateIneqConstraint).isApprox(toVector(other.stateIneqConstraint), prec) &&
         toVector(this->stateInputIneqConstraint).isApprox(toVector(other.stateInputIneqConstraint), prec) &&
         toVector(this->stateEqLagrangian).isApprox(toVector(other.stateEqLagrangian), prec) &&
         toVector(this->stateIneqLagrangian).isApprox(toVector(other.stateIneqLagrangian), prec) &&
         toVector(this->stateInputEqLagrangian).isApprox(toVector(other.stateInputEqLagrangian), prec) &&
         toVector(this->stateInputIneqLagrangian).isApprox(toVector(other.stateInputIneqLagrangian), prec);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t toVector(const std::vector<LagrangianMetrics>& termsLagrangianMetrics) {
  size_t n = 0;
  std::for_each(termsLagrangianMetrics.begin(), termsLagrangianMetrics.end(),
                [&n](const LagrangianMetrics& m) { n += (1 + m.constraint.size()); });

  vector_t vec(n);
  size_t head = 0;
  for (const auto& m : termsLagrangianMetrics) {
    vec(head) = m.penalty;
    vec.segment(head + 1, m.constraint.size()) = m.constraint;
    head += 1 + m.constraint.size();
  }  // end of i loop

  return vec;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t toVector(const vector_array_t& constraintArray) {
  size_t n = 0;
  std::for_each(constraintArray.begin(), constraintArray.end(), [&n](const vector_t& v) { n += v.size(); });

  vector_t vec(n);
  size_t head = 0;
  for (const auto& v : constraintArray) {
    vec.segment(head, v.size()) = v;
    head += v.size();
  }  // end of i loop

  return vec;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_array_t toConstraintArray(const size_array_t& termsSize, const vector_t& vec) {
  vector_array_t constraintArray;
  constraintArray.reserve(termsSize.size());

  size_t head = 0;
  for (const auto& l : termsSize) {
    constraintArray.emplace_back(vec.segment(head, l));
    head += l;
  }  // end of i loop

  return constraintArray;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<LagrangianMetrics> toLagrangianMetrics(const size_array_t& termsSize, const vector_t& vec) {
  std::vector<LagrangianMetrics> lagrangianMetrics;
  lagrangianMetrics.reserve(termsSize.size());

  size_t head = 0;
  for (const auto& l : termsSize) {
    lagrangianMetrics.emplace_back(vec(head), vec.segment(head + 1, l));
    head += 1 + l;
  }  // end of i loop

  return lagrangianMetrics;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_array_t getSizes(const vector_array_t& constraintArray) {
  size_array_t s(constraintArray.size());
  std::transform(constraintArray.begin(), constraintArray.end(), s.begin(),
                 [](const vector_t& v) { return static_cast<size_t>(v.size()); });
  return s;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_array_t getSizes(const std::vector<LagrangianMetrics>& termsLagrangianMetrics) {
  size_array_t s(termsLagrangianMetrics.size());
  std::transform(termsLagrangianMetrics.begin(), termsLagrangianMetrics.end(), s.begin(),
                 [](const LagrangianMetrics& m) { return static_cast<size_t>(m.constraint.size()); });
  return s;
}

}  // namespace ocs2

namespace ocs2 {
namespace LinearInterpolation {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LagrangianMetrics interpolate(const index_alpha_t& indexAlpha, const std::vector<LagrangianMetricsConstRef>& dataArray) {
  const auto penalty =
      interpolate(indexAlpha, dataArray,
                  [](const std::vector<LagrangianMetricsConstRef>& array, size_t t) -> const scalar_t& { return array[t].penalty; });

  const auto constraint =
      interpolate(indexAlpha, dataArray,
                  [](const std::vector<LagrangianMetricsConstRef>& array, size_t t) -> const vector_t& { return array[t].constraint; });

  return {penalty, constraint};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Metrics interpolate(const index_alpha_t& indexAlpha, const std::vector<Metrics>& dataArray) {
  // number of terms
  const auto ind = indexAlpha.second > 0.5 ? indexAlpha.first : indexAlpha.first + 1;
  const size_t numStateEqConst = dataArray[ind].stateEqConstraint.size();
  const size_t numStateInputEqCost = dataArray[ind].stateInputEqConstraint.size();
  const size_t numStateIneqConst = dataArray[ind].stateIneqConstraint.size();
  const size_t numStateInputIneqCost = dataArray[ind].stateInputIneqConstraint.size();
  const size_t numStateEqLag = dataArray[ind].stateEqLagrangian.size();
  const size_t numStateIneqLag = dataArray[ind].stateIneqLagrangian.size();
  const size_t numStateInputEqLag = dataArray[ind].stateInputEqLagrangian.size();
  const size_t numStateInputIneqLag = dataArray[ind].stateInputIneqLagrangian.size();

  Metrics out;

  // cost
  out.cost =
      interpolate(indexAlpha, dataArray, [](const std::vector<Metrics>& array, size_t t) -> const scalar_t& { return array[t].cost; });

  // dynamics violation
  out.dynamicsViolation = interpolate(
      indexAlpha, dataArray, [](const std::vector<Metrics>& array, size_t t) -> const vector_t& { return array[t].dynamicsViolation; });

  // equality constraints
  out.stateEqConstraint.reserve(numStateEqConst);
  for (size_t i = 0; i < numStateEqConst; ++i) {
    auto constraint = interpolate(indexAlpha, dataArray, [i](const std::vector<Metrics>& array, size_t t) -> const vector_t& {
      return array[t].stateEqConstraint[i];
    });
    out.stateEqConstraint.emplace_back(std::move(constraint));
  }
  out.stateInputEqConstraint.reserve(numStateInputEqCost);
  for (size_t i = 0; i < numStateInputEqCost; ++i) {
    auto constraint = interpolate(indexAlpha, dataArray, [i](const std::vector<Metrics>& array, size_t t) -> const vector_t& {
      return array[t].stateInputEqConstraint[i];
    });
    out.stateInputEqConstraint.emplace_back(std::move(constraint));
  }

  // inequality constraints
  out.stateIneqConstraint.reserve(numStateIneqConst);
  for (size_t i = 0; i < numStateIneqConst; ++i) {
    auto constraint = interpolate(indexAlpha, dataArray, [i](const std::vector<Metrics>& array, size_t t) -> const vector_t& {
      return array[t].stateIneqConstraint[i];
    });
    out.stateIneqConstraint.emplace_back(std::move(constraint));
  }
  out.stateInputIneqConstraint.reserve(numStateInputIneqCost);
  for (size_t i = 0; i < numStateInputIneqCost; ++i) {
    auto constraint = interpolate(indexAlpha, dataArray, [i](const std::vector<Metrics>& array, size_t t) -> const vector_t& {
      return array[t].stateInputIneqConstraint[i];
    });
    out.stateInputIneqConstraint.emplace_back(std::move(constraint));
  }

  // state equality Lagrangian
  out.stateEqLagrangian.reserve(numStateEqLag);
  for (size_t i = 0; i < numStateEqLag; ++i) {
    auto penalty = interpolate(indexAlpha, dataArray, [i](const std::vector<Metrics>& array, size_t t) -> const scalar_t& {
      return array[t].stateEqLagrangian[i].penalty;
    });
    auto constraint = interpolate(indexAlpha, dataArray, [i](const std::vector<Metrics>& array, size_t t) -> const vector_t& {
      return array[t].stateEqLagrangian[i].constraint;
    });
    out.stateEqLagrangian.emplace_back(penalty, std::move(constraint));
  }  // end of i loop

  // state inequality Lagrangian
  out.stateIneqLagrangian.reserve(numStateIneqLag);
  for (size_t i = 0; i < numStateIneqLag; ++i) {
    auto penalty = interpolate(indexAlpha, dataArray, [i](const std::vector<Metrics>& array, size_t t) -> const scalar_t& {
      return array[t].stateIneqLagrangian[i].penalty;
    });
    auto constraint = interpolate(indexAlpha, dataArray, [i](const std::vector<Metrics>& array, size_t t) -> const vector_t& {
      return array[t].stateIneqLagrangian[i].constraint;
    });
    out.stateIneqLagrangian.emplace_back(penalty, std::move(constraint));
  }  // end of i loop

  // state-input equality Lagrangian
  out.stateInputEqLagrangian.reserve(numStateInputEqLag);
  for (size_t i = 0; i < numStateInputEqLag; ++i) {
    auto penalty = interpolate(indexAlpha, dataArray, [i](const std::vector<Metrics>& array, size_t t) -> const scalar_t& {
      return array[t].stateInputEqLagrangian[i].penalty;
    });
    auto constraint = interpolate(indexAlpha, dataArray, [i](const std::vector<Metrics>& array, size_t t) -> const vector_t& {
      return array[t].stateInputEqLagrangian[i].constraint;
    });
    out.stateInputEqLagrangian.emplace_back(penalty, std::move(constraint));
  }  // end of i loop

  // state-input inequality Lagrangian
  out.stateInputIneqLagrangian.reserve(numStateInputIneqLag);
  for (size_t i = 0; i < numStateInputIneqLag; ++i) {
    auto penalty = interpolate(indexAlpha, dataArray, [i](const std::vector<Metrics>& array, size_t t) -> const scalar_t& {
      return array[t].stateInputIneqLagrangian[i].penalty;
    });
    auto constraint = interpolate(indexAlpha, dataArray, [i](const std::vector<Metrics>& array, size_t t) -> const vector_t& {
      return array[t].stateInputIneqLagrangian[i].constraint;
    });
    out.stateInputIneqLagrangian.emplace_back(penalty, std::move(constraint));
  }  // end of i loop

  return out;
}

}  // namespace LinearInterpolation
}  // namespace ocs2
