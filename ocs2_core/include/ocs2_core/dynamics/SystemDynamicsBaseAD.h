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
#include <ocs2_core/dynamics/SystemDynamicsBase.h>

namespace ocs2 {

/**
 * The system dynamics Base with Algorithmic Differentiation (i.e. Auto Differentiation).
 * The linearized system flow map is defined as: \n
 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
 * The linearized system jump map is defined as: \n
 * \f$ x^+ = G \delta x + H \delta u \f$ \n
 */
class SystemDynamicsBaseAD : public SystemDynamicsBase {
 public:
  /** Constructor */
  SystemDynamicsBaseAD();

  /** Default destructor */
  ~SystemDynamicsBaseAD() override = default;

  /**
   * Initializes model libraries
   *
   * @param stateDim : state vector dimension.
   * @param inputDim : input vector dimension.
   * @param modelName : name of the generate model library
   * @param modelFolder : folder to save the model library files to
   * @param recompileLibraries : If true, always compile the model library, else try to load existing library if available.
   * @param verbose : print information.
   */
  void initialize(size_t stateDim, size_t inputDim, const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2",
                  bool recompileLibraries = true, bool verbose = true);

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComputation) final;

  vector_t computeJumpMap(scalar_t t, const vector_t& x, const PreComputation& preComputation) final;

  vector_t computeGuardSurfaces(scalar_t t, const vector_t& x) final;

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation& preComputation) final;

  VectorFunctionLinearApproximation jumpMapLinearApproximation(scalar_t t, const vector_t& x, const PreComputation& preComputation) final;

  VectorFunctionLinearApproximation guardSurfacesLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u) final;

  /** @note: Requires linear approximation to be called before */
  vector_t flowMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) final;

  /** @note: Requires jump map linear approximation to be called before */
  vector_t jumpMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) final;

  /** @note: Requires guard surfaces linear approximation to be called before */
  vector_t guardSurfacesDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) final;

 protected:
  /** Copy constructor */
  SystemDynamicsBaseAD(const SystemDynamicsBaseAD& rhs);

  /**
   * Interface method to the state flow map of the hybrid system. This method should be implemented by the derived class.
   *
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] input: input vector.
   * @param [in] parameters: parameter vector.
   * @return state vector time derivative.
   */
  virtual ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                    const ad_vector_t& parameters) const = 0;

  /**
   * Gets the parameters of the system flow map
   *
   * @param [in] time: Current time.
   * @return The parameters to be set in the flow map at the start of the horizon
   */
  virtual vector_t getFlowMapParameters(scalar_t time, const PreComputation& /* preComputation */) const { return vector_t(0); }

  /**
   * Number of parameters for system flow map.
   *
   * @return number of parameters
   */
  virtual size_t getNumFlowMapParameters() const { return 0; }

  /**
   * Interface method to the state jump map of the hybrid system. This method can be implemented by the derived class.
   *
   * @param [in] time: time.
   * @param [in] state: state vector.
   * @param [in] parameters: parameter vector.
   * @return jumped state.
   */
  virtual ad_vector_t systemJumpMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const;

  /**
   * Gets the parameters of the jump map
   *
   * @param [in] time: Current time.
   * @return The parameters to be set in the jump map
   */
  virtual vector_t getJumpMapParameters(scalar_t time, const PreComputation& /* preComputation */) const { return vector_t(0); }

  /**
   * Number of parameters for jump map.
   *
   * @return number of parameters
   */
  virtual size_t getNumJumpMapParameters() const { return 0; }

  /**
   * Interface method to the guard surfaces. This method can be implemented by the derived class.
   *
   * @param [in] time: time.
   * @param [in] state: state.
   * @param [in] input: input vector
   * @param [in] parameters: parameter vector.
   * @return A vector of guard surfaces values
   */
  virtual ad_vector_t systemGuardSurfaces(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const;

  /**
   * Gets the parameters of the guard surfaces
   *
   * @param [in] time: Current time.
   * @return The parameters to be set in the guard surfaces
   */
  virtual vector_t getGuardSurfacesParameters(scalar_t time) const { return vector_t(0); }

  /**
   * Number of parameters for guard surfaces.
   *
   * @return number of parameters
   */
  virtual size_t getNumGuardSurfacesParameters() const { return 0; }

 private:
  std::unique_ptr<CppAdInterface> flowMapADInterfacePtr_;
  std::unique_ptr<CppAdInterface> jumpMapADInterfacePtr_;
  std::unique_ptr<CppAdInterface> guardSurfacesADInterfacePtr_;

  vector_t tapedTimeStateInput_;
  vector_t tapedTimeState_;

  /** Cached jacobians for time derivative */
  matrix_t flowJacobian_;
  matrix_t jumpJacobian_;
  matrix_t guardJacobian_;
};

}  // namespace ocs2
