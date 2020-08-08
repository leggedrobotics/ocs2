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

#include <memory>

// GNU Linear Programming Kit
#include <glpk.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_frank_wolfe/NLP_Constraints.h>

namespace ocs2 {

/**
 * This class implements the Frank-Wolfe algorithm for computing the descent direction
 * respecting linear equalities and inequalities. For more discussion on this
 * algorithm, the reader should refer to \cite jaggi13 .
 */
class FrankWolfeDescentDirection {
 public:
  /**
   * Constructor.
   *
   * @param [in] display:
   */
  explicit FrankWolfeDescentDirection(bool display);

  /**
   * Default destructor.
   */
  ~FrankWolfeDescentDirection() = default;

  /**
   * Calculates the Frank-Wolfe descent direction which respects the domain
   * constraints as well as element-wise maximum of the descent direction
   * which defined as: \n
   * \f$ | diag[e_v] d_v | \leq 1 0\f$.
   *
   * Note that if there is no limit for a direction set associated
   * element of \f$ e_v \f$ to zero.
   *
   * @param [in] parameter: The value of parameter vector.
   * @param [in] gradient: The gradient at the current parameter vector.
   * @param [in] maxGradientInverse: descent directions element-wise maximum inverse, \f$ e_v \f$.
   * @param [in] nlpConstraintsPtr: A pointer to the NLP constraints.
   * @param [out] fwDescentDirection: The Frank-Wolfe descent direction at the current parameter vector.
   */
  void run(const vector_t& parameter, const vector_t& gradient, const vector_t& maxGradientInverse, NLP_Constraints* nlpConstraintsPtr,
           vector_t& fwDescentDirection);

 private:
  /**
   * Instantiates GLPK solver
   */
  void instantiateGLPK();

  /**
   * Sets up Frank-Wolfe linear program.
   *
   * @param [in] parameter: The value of parameter vector.
   * @param [in] gradient: The gradient at the current parameter vector.
   * @param [in] maxGradientInverse: descent directions element-wise maximum inverse, \f$ e_v \f$.
   * @param [in] nlpConstraintsPtr: A pointer to the NLP constraints.
   */
  void setupLP(const vector_t& parameter, const vector_t& gradient, const vector_t& maxGradientInverse, NLP_Constraints* nlpConstraintsPtr);

  /***********
   * Variables
   **********/
  std::unique_ptr<glp_prob, void (*)(glp_prob*)> lpPtr_;
  std::unique_ptr<glp_smcp> lpOptionsPtr_;
};

}  // namespace ocs2
