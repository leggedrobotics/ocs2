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

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <ocs2_core/Types.h>
#include "ocs2_frank_wolfe/FrankWolfeDescentDirection.h"
#include "ocs2_frank_wolfe/NLP_Constraints.h"
#include "ocs2_frank_wolfe/NLP_Cost.h"
#include "ocs2_frank_wolfe/NLP_Settings.h"

namespace ocs2 {

/**
 * This class implements the Frank-Wolfe algorithm which is an iterative first-order gradient descent
 * algorithm. For more discussion on this algorithm, the reader should refer to
 * \cite jaggi13 .
 *
 * User should override three methods namely:
 * calculateCost, calculateGradient, calculateLinearEqualityConstraint, and getSolution.
 *
 */
class GradientDescent {
 public:
  /**
   * Constructor.
   *
   * @param [in] nlpSettings: Structure containing the settings for the NLP algorithm.
   */
  explicit GradientDescent(const NLP_Settings& nlpSettings);

  /**
   * Default destructor.
   */
  virtual ~GradientDescent() = default;

  /**
   * Gets the cost.
   *
   * @param [out] cost value
   */
  void getCost(scalar_t& cost) const;

  /**
   * Gets the parameter vector.
   *
   * @param [out] parameters: the parameter vector
   */
  void getParameters(vector_t& parameters) const;

  /**
   * Gets the iteration cost log.
   *
   * @param [out] iterationCost: The cost value in each iteration.
   */
  void getIterationsLog(scalar_array_t& iterationCost) const;

  /**
   * Gets a constant reference to the optimal solver's ID.
   *
   * @param [out] optimizedID: The optimal solver's ID.
   */
  void optimalSolutionID(size_t& optimizedID) const;

  /**
   * Gets a constant reference to the NLP of settings.
   *
   * @return A constant reference to the NLP of settings.
   */
  NLP_Settings& nlpSettings();

  /**
   * This method runs the Frank-Wolfe algorithm which the initial parameters
   * which respects the domain constraints as well as element-wise maximum of
   * the descent direction which defined as: \n
   * \f$ | diag[e_v] d_v | \leq 1 0\f$.
   *
   * Note that if there is no limit for a direction set associated
   * element of \f$ e_v \f$ to zero.
   *
   * @param [in] initParameters: The value of parameter vector.
   * @param [in] maxGradientInverse: descent directions element-wise maximum inverse, \f$ e_v \f$.
   * @param [in] costPtr: A pointer to the NLP cost.
   * @param [in] constraintsPtr: A pointer to the NLP constraints.
   */
  void run(const vector_t& initParameters, const vector_t& maxGradientInverse, NLP_Cost* costPtr,
           NLP_Constraints* constraintsPtr = nullptr);

 protected:
  /**
   * Line search to find the best learning rate using decreasing scheme where the step size eventually decreases
   * from the maximum value to the minimum.
   *
   * @param [in] parameters: The current parameter vector.
   * @param [in] gradient: The current gradient.
   * @param [in] costPtr: A pointer to the NLP cost.
   * @param [in] constraintsPtr: A pointer to the NLP constraints.
   * @param [out] optimizedParameters: The parameter vector.
   * @param [out] optimizedCost: The optimized cost.
   * @param [out] optimizedID: The ID of the optimized solution.
   * @param [out] optimizedLearningRate: The optimized learning rate.
   */
  void lineSearch(const vector_t& parameters, const vector_t& gradient, NLP_Cost* costPtr, NLP_Constraints* constraintsPtr,
                  vector_t& optimizedParameters, scalar_t& optimizedCost, size_t& optimizedID, scalar_t& optimizedLearningRate);

  /*
   * Variables
   */
  NLP_Settings nlpSettings_;
  std::unique_ptr<FrankWolfeDescentDirection> frankWolfeDescentDirectionPtr_;

  scalar_t optimizedCost_;
  size_t optimizedID_;
  vector_t optimizedParameters_;
  vector_t optimizedGradient_;
  size_t numFuntionCall_;

  scalar_array_t iterationCost_;

  Eigen::IOFormat CleanFmtDisplay_;
};

}  // namespace ocs2
