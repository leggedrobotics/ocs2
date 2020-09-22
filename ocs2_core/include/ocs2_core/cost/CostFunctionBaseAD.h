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

#pragma once

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include "ocs2_core/cost/CostFunctionBase.h"

namespace ocs2 {

/**
 * Cost Function Base with Algorithmic Differentiation (i.e. Auto Differentiation).
 */
class CostFunctionBaseAD : public CostFunctionBase {
 public:
  using ad_scalar_t = typename ocs2::ad_scalar_t;
  using ad_vector_t = typename ocs2::ad_vector_t;

  /** Default constructor */
  CostFunctionBaseAD(size_t stateDim, size_t inputDim);

  /** Default destructor */
  ~CostFunctionBaseAD() override = default;

  /**
   * Initializes model libraries
   *
   * @param modelName : name of the generate model library
   * @param modelFolder : folder to save the model library files to
   * @param recompileLibraries : If true, the model library will be newly compiled. If false, an existing library will be loaded if
   * available.
   * @param verbose : print information.
   */
  void initialize(const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true,
                  bool verbose = true);

  scalar_t cost(scalar_t t, const vector_t& x, const vector_t& u) final;
  scalar_t finalCost(scalar_t t, const vector_t& x) final;
  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) final;
  ScalarFunctionQuadraticApproximation finalCostQuadraticApproximation(scalar_t t, const vector_t& x) final;

  /** @note: Requires cost quadratic approximation to be called before */
  scalar_t costDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) final;

  /** @note: Requires final cost quadratic approximation to be called before */
  scalar_t finalCostDerivativeTime(scalar_t t, const vector_t& x) final;

 protected:
  /** Copy constructor */
  CostFunctionBaseAD(const CostFunctionBaseAD& rhs);

  /**
   * Gets a user-defined cost parameters, applied to the intermediate costs
   *
   * @param [in] time: Current time.
   * @return The cost function parameters at a certain time
   */
  virtual vector_t getIntermediateParameters(scalar_t time) const { return vector_t(0); }

  /**
   * Number of parameters for the intermediate cost function.
   * This number must be remain constant after the model libraries are created
   *
   * @return number of parameters
   */
  virtual size_t getNumIntermediateParameters() const { return 0; }

  /**
   * Gets a user-defined cost parameters, applied to the final costs
   *
   * @param [in] time: Current time.
   * @return The cost function parameters at a certain time
   */
  virtual vector_t getFinalParameters(scalar_t time) const { return vector_t(0); }

  /**
   * Number of parameters for the final cost function.
   * This number must be remain constant after the model libraries are created
   *
   * @return number of parameters
   */
  virtual size_t getNumFinalParameters() const { return 0; }

  /**
   * Interface method to the intermediate cost function. This method must be implemented by the derived class.
   *
   * @tparam scalar type. All the floating point operations should be with this type.
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] input: input vector.
   * @param [in] parameters: parameter vector.
   * @return cost value.
   */
  virtual ad_scalar_t intermediateCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                               const ad_vector_t& parameters) const = 0;

  /**
   * Interface method to the final cost function. This method can be implemented by the derived class.
   *
   * @tparam scalar type. All the floating point operations should be with this type.
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] parameters: parameter vector.
   * @return cost value.
   */
  virtual ad_scalar_t finalCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const {
    return ad_scalar_t(0);
  }

 private:
  /**
   * Sets all the required CppAdCodeGenInterfaces
   */
  void setADInterfaces(const std::string& modelName, const std::string& modelFolder);

  /**
   * Create the forward model and derivatives.
   *
   * @param [in] verbose: display information.
   */
  void createModels(bool verbose);

  /**
   * Loads the forward model and derivatives if available. Constructs them otherwise.
   *
   * @param [in] verbose: display information
   */
  void loadModelsIfAvailable(bool verbose);

 protected:
  size_t stateDim_;
  size_t inputDim_;

 private:
  std::unique_ptr<CppAdInterface> finalADInterfacePtr_;
  std::unique_ptr<CppAdInterface> intermediateADInterfacePtr_;

  // Intermediate cost
  vector_t intermediateParameters_;
  vector_t tapedTimeStateInput_;
  row_vector_t intermediateJacobian_;
  matrix_t intermediateHessian_;

  // Final cost
  vector_t finalParameters_;
  vector_t tapedTimeState_;
  row_vector_t finalJacobian_;
  matrix_t finalHessian_;
};

}  // namespace ocs2
