/******************************************************************************
Copyright (c) 2020, Johannes Pankert. All rights reserved.

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
#include <ocs2_core/cost/CostFunctionBase.h>

namespace ocs2 {

/**
 *
 * Relaxed Barrier Soft Constraint Cost with Algorithmic Differentiation (i.e. Auto Differentiation).
 *
 * The intermediate cost term and the final cost term have the following form:
 * - \f$ L = RelaxedBarrierFunction(f(x,u,t)) \f$
 * - \f$ \Phi = RelaxedBarrierFunction(f(x,u,t)) \f$.
 *
 * The RelaxedBarrierFunction applies -mu*ln(z) element wise to the user provided function outputs. If z<delta
 * the a quadratic function is applied instead. The quadratic function is parameterized such that the concatenation
 * with -mu*ln(x) is continuously differentiable.
 * The user provides implementations for f and g. The Jacobians of f and g are computed by auto differentiation.
 * This costfunction approximates the Hessians of L and \Phi by applying the chain rule and neglecting the terms
 * with hessians of f and g. Hess_{L} and H_{\Phi} are therefore guaranteed to be at least positive semidefinite.
 */
class RelaxedBarrierCost : public CostFunctionBase {
 public:
  using ad_scalar_t = typename CppAdInterface::ad_scalar_t;
  using ad_vector_t = typename CppAdInterface::ad_vector_t;

  /**
   * Configuration object
   *
   * mu : scaling factor
   * delta: relaxation parameter, see class description
   */
  struct Config {
    scalar_t mu;
    scalar_t delta;
  };

  /**
   * Constructior
   *
   * @param Config : configuration object containing mu and delta
   */
  RelaxedBarrierCost(Config config, size_t stateDim, size_t inputDim, size_t intermediateCostDim, size_t finalCostDim);

  /**
   * Constructior
   *
   * @param Config : arrays with configuration objects containing mu and delta for each intermediate and final constraint
   */
  explicit RelaxedBarrierCost(std::vector<Config> intermediateConfig, std::vector<Config> finalConfig, size_t stateDim, size_t inputDim);

  /** Default destructor */
  virtual ~RelaxedBarrierCost() = default;

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

  scalar_t cost(scalar_t t, const vector_t& x, const vector_t& u) override;
  scalar_t finalCost(scalar_t t, const vector_t& x) override;
  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) override;
  ScalarFunctionQuadraticApproximation finalCostQuadraticApproximation(scalar_t t, const vector_t& x) override;
  scalar_t costDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) override;
  scalar_t finalCostDerivativeTime(scalar_t t, const vector_t& x) override;

 protected:
  /** Copy constructor */
  RelaxedBarrierCost(const RelaxedBarrierCost& rhs);

  /**
   * Interface method to the cost term f such that the intermediate cost is
   * - \f$ L = RelaxedBarrierFunction(f(x,u,t)) \f$
   * This method must be implemented by the derived class.
   *
   * @tparam scalar type. All the floating point operations should be with this type.
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] input: input vector.
   * @param [in] parameters: parameter vector.
   * @param [out] costValue: costValues = f(x,u,t).
   */
  virtual vector_t getIntermediateParameters(scalar_t time) const;

  /**
   * Number of parameters for the intermediate cost function.
   * This number must be remain constant after the model libraries are created
   *
   * @return number of parameters
   */
  virtual size_t getNumIntermediateParameters() const;

  /**
   * Interface method to the cost term g such that the intermediate cost is
   * - \f$ \Phi = RelaxedBarrierFunction(f(x,u,t)) \f$.
   * This method may be implemented by the derived class.
   *
   * @tparam scalar type. All the floating point operations should be with this type.
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] parameters: parameter vector.
   * @param [out] costValue: costValues = g(x,t).
   */
  virtual vector_t getFinalParameters(scalar_t time) const;

  /**
   * Number of parameters for the final cost function.
   * This number must be remain constant after the model libraries are created
   *
   * @return number of parameters
   */
  virtual size_t getNumFinalParameters() const;

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
  virtual ad_vector_t intermediateCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
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
  virtual ad_vector_t finalCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const;

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

  scalar_t getPenaltyFunctionValue(scalar_t h, const Config& config) const;
  scalar_t getPenaltyFunctionDerivative(scalar_t h, const Config& config) const;
  scalar_t getPenaltyFunctionSecondDerivative(scalar_t h, const Config& config) const;

  std::unique_ptr<CppAdInterface> finalADInterfacePtr_;
  std::unique_ptr<CppAdInterface> intermediateADInterfacePtr_;

  size_t stateDim_;
  size_t inputDim_;

  // Intermediate cost
  vector_t intermediateCostValues_;
  matrix_t intermediateJacobian_;
  vector_t intermediateParameters_;
  vector_t tapedTimeStateInput_;

  // Final cost
  vector_t finalCostValues_;
  matrix_t finalJacobian_;
  vector_t finalParameters_;
  vector_t tapedTimeState_;

  std::vector<Config> intermediateConfig_;
  std::vector<Config> finalConfig_;
};

}  // namespace ocs2
