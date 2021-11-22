/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include "ocs2_core/loopshaping/LoopshapingPropertyTree.h"

#include <boost/property_tree/info_parser.hpp>
#include "ocs2_core/misc/LoadData.h"

namespace ocs2 {
namespace loopshaping_property_tree {
Filter readSISOFilter(const boost::property_tree::ptree& pt, std::string filterName, bool invert) {
  // Get Sizes
  const auto numRepeats = pt.get<size_t>(filterName + ".numRepeats", 1);
  const auto numPoles = pt.get<size_t>(filterName + ".numPoles", 0);
  const auto numZeros = pt.get<size_t>(filterName + ".numZeros", 0);
  const auto scaling = pt.get<scalar_t>(filterName + ".scaling", 1.0);
  const size_t numStates = invert ? numZeros : numPoles;
  const size_t numInputs = 1;

  // Setup Filter, convention a0*s^n + a1*s^(n-1) + ... + an
  vector_t numerator(numZeros + 1);
  numerator.setZero();
  numerator(0) = 1.0;
  for (size_t z = 0; z < numZeros; z++) {
    auto zero = pt.get<scalar_t>(filterName + ".zeros." + "(" + std::to_string(z) + ")");
    numerator.segment(1, z + 1) -= zero * numerator.segment(0, z + 1).eval();
  }
  numerator *= scaling;  // Apply scale to numerator

  vector_t denominator(numPoles + 1);
  denominator.setZero();
  denominator(0) = 1.0;
  for (size_t p = 0; p < numPoles; p++) {
    auto pole = pt.get<scalar_t>(filterName + ".poles." + "(" + std::to_string(p) + ")");
    denominator.segment(1, p + 1) -= pole * denominator.segment(0, p + 1).eval();
  }

  // Print frequency domain information
  std::cerr << "Read filter " << filterName << (invert ? " (before inversion)" : "") << ", convention: a0*s^n + a1*s^(n-1) + ... + an \n";
  std::cerr << "\tnumerator: [" << numerator.transpose() << "]\n";
  std::cerr << "\tdenominator: [" << denominator.transpose() << "]\n";
  std::cerr << "\tDC gain: " << numerator(numZeros) / denominator(numPoles) << "\n";
  std::cerr << "\tInf gain: ";
  if (numZeros > numPoles) {
    std::cerr << "Inf\n";
  }
  if (numZeros < numPoles) {
    std::cerr << 0.0 << "\n";
  }
  if (numZeros == numPoles) {
    std::cerr << numerator(0) / denominator(0) << "\n";
  }

  if (invert) {
    vector_t temp;
    temp = numerator;
    numerator = denominator;
    denominator = temp;
  }

  // Convert to state space
  matrix_t a, b, c, d;
  ocs2::tf2ss(numerator, denominator, a, b, c, d);

  matrix_t A = matrix_t::Zero(numRepeats * numStates, numRepeats * numStates);
  matrix_t B = matrix_t::Zero(numRepeats * numStates, numRepeats * numInputs);
  matrix_t C = matrix_t::Zero(numRepeats * numInputs, numRepeats * numStates);
  matrix_t D = matrix_t::Zero(numRepeats * numInputs, numRepeats * numInputs);
  for (size_t r = 0; r < numRepeats; r++) {
    A.block(r * numStates, r * numStates, numStates, numStates) = a;
    B.block(r * numStates, r * numInputs, numStates, numInputs) = b;
    C.block(r * numInputs, r * numStates, numInputs, numStates) = c;
    D.block(r * numInputs, r * numInputs, numInputs, numInputs) = d;
  }

  return Filter(A, B, C, D);
}

Filter readMIMOFilter(const boost::property_tree::ptree& pt, std::string filterName, bool invert) {
  const auto numFilters = pt.get<size_t>(filterName + ".numFilters", 0);
  if (numFilters > 0) {
    // Read the sisoFilters
    std::vector<Filter> sisoFilters;
    size_t numStates(0), numInputs(0), numOutputs(0);
    for (size_t i = 0; i < numFilters; ++i) {
      // Read filter
      std::string sisoFilterName = filterName + ".Filter" + std::to_string(i);
      sisoFilters.emplace_back(readSISOFilter(pt, sisoFilterName, invert));

      // Track sizes
      numStates += sisoFilters.back().getNumStates();
      numInputs += sisoFilters.back().getNumInputs();
      numOutputs += sisoFilters.back().getNumOutputs();
    }

    // Concatenate siso matrices into one MIMO filter
    matrix_t A = matrix_t::Zero(numStates, numStates);
    matrix_t B = matrix_t::Zero(numStates, numInputs);
    matrix_t C = matrix_t::Zero(numOutputs, numStates);
    matrix_t D = matrix_t::Zero(numOutputs, numInputs);
    size_t statecount(0), inputcount(0), outputcount(0);
    for (const auto& filt : sisoFilters) {
      A.block(statecount, statecount, filt.getNumStates(), filt.getNumStates()) = filt.getA();
      B.block(statecount, inputcount, filt.getNumStates(), filt.getNumInputs()) = filt.getB();
      C.block(outputcount, statecount, filt.getNumOutputs(), filt.getNumStates()) = filt.getC();
      D.block(outputcount, inputcount, filt.getNumOutputs(), filt.getNumInputs()) = filt.getD();
      statecount += filt.getNumStates();
      inputcount += filt.getNumInputs();
      outputcount += filt.getNumOutputs();
    }
    return Filter(A, B, C, D);
  } else {
    return Filter();
  }
}

std::shared_ptr<LoopshapingDefinition> load(const std::string& settingsFile) {
  // Read from settings File
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(settingsFile, pt);
  Filter r_filter = loopshaping_property_tree::readMIMOFilter(pt, "r_filter");
  Filter s_filter = loopshaping_property_tree::readMIMOFilter(pt, "s_inv_filter", /*invert=*/true);

  if (r_filter.getNumOutputs() > 0 && s_filter.getNumOutputs() > 0) {
    throw std::runtime_error("[LoopshapingDefinition] using both r and s filter not implemented");
  }

  if (r_filter.getNumOutputs() > 0) {
    return std::make_shared<LoopshapingDefinition>(LoopshapingType::outputpattern, r_filter);
  }
  if (s_filter.getNumOutputs() > 0) {
    return std::make_shared<LoopshapingDefinition>(LoopshapingType::eliminatepattern, s_filter);
  }

  throw std::runtime_error("[LoopshapingDefinition] error loading loopshaping definition, no valid filter found");
}

}  // namespace loopshaping_property_tree
}  // namespace ocs2
