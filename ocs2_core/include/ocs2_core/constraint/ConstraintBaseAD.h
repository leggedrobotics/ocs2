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

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/constraint/ConstraintBase.h>

namespace ocs2 {

/**
 * The system constraint Base with Algorithmic Differentiation (i.e. Auto Differentiation).
 * The linearized state-input constraint is defined as: \n
 * \f$ 0 = C(t) \delta x + D(t) \delta u + e(t) \f$ \n
 * The linearized state-only constraint is defined as: \n
 * \f$ 0 = F(t) \delta x + h(t) \f$ \n
 * The linearized final state-only constraint is defined as: \n
 * \f$ 0 = F_f \delta x + h_f \f$ \n
 */
class ConstraintBaseAD : public ConstraintBase {
 public:
  using ad_scalar_t = typename CppAdInterface::ad_scalar_t;
  using ad_vector_t = typename CppAdInterface::ad_vector_t;

  /**
   * Constructor.
   *
   * @param[in] stateDim: State vector dimension
   * @param[in] inputDim: Input vector dimension
   */
  explicit ConstraintBaseAD(size_t stateDim, size_t inputDim);

  /**
   * Copy constructor
   */
  ConstraintBaseAD(const ConstraintBaseAD& rhs);

  /**
   * Default destructor
   */
  ~ConstraintBaseAD() override = default;

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

  void setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) final;

  vector_t getStateInputEqualityConstraint() final;

  vector_t getStateEqualityConstraint() final;

  vector_t getFinalStateEqualityConstraint() final;

  matrix_t getStateInputEqualityConstraintDerivativesState() final;

  matrix_t getStateInputEqualityConstraintDerivativesInput() final;

  matrix_t getStateEqualityConstraintDerivativesState() final;

  matrix_t getFinalStateEqualityConstraintDerivativesState() final;

 protected:
  /**
   * Interface method to the state-input equality constraints. This method should be implemented by the derived class.
   *
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] input: input vector
   * return constraints vector.
   */
  virtual ad_vector_t stateInputConstraint(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input) const;

  /**
   * Interface method to the state-only equality constraints. This method should be implemented by the derived class.
   *
   * @param [in] time: time.
   * @param [in] state: state vector.
   * return constraint vector.
   */
  virtual ad_vector_t stateOnlyConstraint(ad_scalar_t time, const ad_vector_t& state) const;

  /**
   * Interface method to the state-only final equality constraints. This method should be implemented by the derived class.
   *
   * @param [in] time: time.
   * @param [in] state: state vector.
   * return constraint vector.
   */
  virtual ad_vector_t stateOnlyFinalConstraint(ad_scalar_t time, const ad_vector_t& state) const;

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

  std::unique_ptr<CppAdInterface> stateInputADInterfacePtr_;
  std::unique_ptr<CppAdInterface> stateOnlyADInterfacePtr_;
  std::unique_ptr<CppAdInterface> stateOnlyFinalADInterfacePtr_;

  matrix_t stateInputJacobian_;
  matrix_t stateOnlyJacobian_;
  matrix_t stateOnlyFinalJacobian_;
  vector_t stateInputValues_;
  vector_t stateOnlyValues_;
  vector_t stateOnlyFinalValues_;
};

}  // namespace ocs2
