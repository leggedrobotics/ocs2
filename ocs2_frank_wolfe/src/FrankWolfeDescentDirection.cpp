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

#include <ocs2_frank_wolfe/FrankWolfeDescentDirection.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrankWolfeDescentDirection::FrankWolfeDescentDirection(bool display)
    : lpPtr_(glp_create_prob(), glp_delete_prob), lpOptionsPtr_(new glp_smcp) {
  // set LP options
  glp_init_smcp(lpOptionsPtr_.get());
  if (!display) lpOptionsPtr_->msg_lev = GLP_MSG_ERR;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FrankWolfeDescentDirection::instantiateGLPK() {
  // erase the solver
  glp_erase_prob(lpPtr_.get());

  // name
  glp_set_prob_name(lpPtr_.get(), "FrankWolfe");

  // set it as a minimization problem
  glp_set_obj_dir(lpPtr_.get(), GLP_MIN);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FrankWolfeDescentDirection::setupLP(const vector_t& parameter, const vector_t& gradient, const vector_t& maxGradientInverse,
                                         NLP_Constraints* nlpConstraintsPtr) {
  const size_t parameterDim = parameter.size();

  // return if there is no parameter
  if (parameterDim == 0) return;

  // set parameters limits
  glp_add_cols(lpPtr_.get(), parameterDim);

  // set the LP cost function of Frank-Wolfe algorithm
  for (size_t i = 0; i < parameterDim; i++) glp_set_obj_coef(lpPtr_.get(), i + 1, gradient(i));

  // set descent directions reciprocal element-wise max
  const vector_t Ev = maxGradientInverse.cwiseAbs();
  for (size_t i = 0; i < parameterDim; i++) {
    // if the gradient is zero in one direction
    if (numerics::almost_eq(gradient(i), 0.0)) {
      glp_set_col_bnds(lpPtr_.get(), i + 1, GLP_FX, 0.0, 0.0);

      // if the gradient should be limited
    } else if (!numerics::almost_eq(Ev(i), 0.0)) {
      glp_set_col_bnds(lpPtr_.get(), i + 1, GLP_DB, -1.0 / Ev(i), 1.0 / Ev(i));

      // if free
    } else {
      glp_set_col_bnds(lpPtr_.get(), i + 1, GLP_FR, 0.0, 0.0);
    }

  }  // end of i loop

  // set the current parameter vector.
  nlpConstraintsPtr->setCurrentParameter(parameter);

  // get domain equality constraints
  vector_t g;
  nlpConstraintsPtr->getLinearEqualityConstraint(g);
  matrix_t dgdx;
  nlpConstraintsPtr->getLinearEqualityConstraintDerivative(dgdx);
  if (g.size() > 0 && dgdx.cols() != parameterDim)
    throw std::runtime_error(
        "calculateLinearEqualityConstraint: The number of columns of Jacobian matrix "
        "should be equal to the number of parameters.");
  if (g.size() > 0 && dgdx.rows() != g.rows())
    throw std::runtime_error(
        "calculateLinearEqualityConstraint: The number of rows of Jacobian matrix "
        "should be equal to the number of equality constraints.");

  // get domain inequality constraints
  vector_t h;
  nlpConstraintsPtr->getLinearInequalityConstraint(h);
  matrix_t dhdx;
  nlpConstraintsPtr->getLinearInequalityConstraintDerivative(dhdx);
  if (h.size() > 0 && dhdx.cols() != parameterDim)
    throw std::runtime_error(
        "calculateLinearInequalityConstraint: The number of columns of Jacobian matrix "
        "should be equal to the number of parameters.");
  if (h.size() > 0 && dhdx.rows() != h.rows())
    throw std::runtime_error(
        "calculateLinearInequalityConstraint: The number of rows of Jacobian matrix "
        "should be equal to the number of inequality constraints.");

  // set the total number of constraint limits
  glp_add_rows(lpPtr_.get(), g.size() + h.size());
  scalar_array_t values{0.1};     // 0 index is not used!
  std::vector<int> xIndices{-1};  // 0 index is not used!
  std::vector<int> yIndices{-1};  // 0 index is not used!

  // domain equality constraints
  for (size_t i = 0; i < g.size(); i++) {
    for (size_t j = 0; j < parameterDim; j++) {
      if (!numerics::almost_eq(dgdx(i, j), 0.0)) {
        values.push_back(dgdx(i, j));
        xIndices.push_back(i + 1);
        yIndices.push_back(j + 1);
      }
    }
    glp_set_row_bnds(lpPtr_.get(), i + 1, GLP_FX, -g(i), -g(i));
  }

  // domain inequality constraints
  for (size_t i = 0; i < h.size(); i++) {
    for (size_t j = 0; j < parameterDim; j++) {
      if (!numerics::almost_eq(dhdx(i, j), 0.0)) {
        values.push_back(dhdx(i, j));
        xIndices.push_back(i + 1);
        yIndices.push_back(j + 1);
      }
    }
    glp_set_row_bnds(lpPtr_.get(), i + 1, GLP_LO, -h(i), 0.0);
  }

  // set the constraint coefficients
  glp_load_matrix(lpPtr_.get(), g.size() + h.size(), xIndices.data(), yIndices.data(), values.data());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FrankWolfeDescentDirection::run(const vector_t& parameter, const vector_t& gradient, const vector_t& maxGradientInverse,
                                     NLP_Constraints* nlpConstraintsPtr, vector_t& fwDescentDirection) {
  if (gradient.size() != parameter.size()) throw std::runtime_error("The gradient vector size is incompatible to the parameter size.");
  if (maxGradientInverse.size() != gradient.size())
    throw std::runtime_error("The gradient limit size is incompatible to the gradient size.");

  // instantiate GLPK
  instantiateGLPK();

  // setup LP
  setupLP(parameter, gradient, maxGradientInverse, nlpConstraintsPtr);

  // solve LP
  glp_simplex(lpPtr_.get(), lpOptionsPtr_.get());

  // get the solution
  fwDescentDirection.resize(parameter.size());
  for (size_t i = 0; i < parameter.size(); i++) fwDescentDirection(i) = glp_get_col_prim(lpPtr_.get(), i + 1);

  // test
  if (gradient.dot(fwDescentDirection) > 0) throw std::runtime_error("Frank-Wolfe does not produce a descent direction.");
}

}  // namespace ocs2
