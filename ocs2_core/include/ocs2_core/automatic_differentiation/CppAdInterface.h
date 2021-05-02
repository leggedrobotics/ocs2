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

// Eigen
#include <Eigen/Core>

// STL
#include <string>

// CppAD
#include <cppad/cg.hpp>

// CppAD helpers
#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/CppAdSparsity.h>
#include <ocs2_core/automatic_differentiation/Types.h>

namespace ocs2 {

class CppAdInterface {
 public:
  enum class ApproximationOrder { Zero, First, Second };

  using ad_base_t = ocs2::ad_base_t;
  using ad_scalar_t = ocs2::ad_scalar_t;
  using ad_vector_t = ocs2::ad_vector_t;
  using ad_function_t = std::function<void(const ad_vector_t&, ad_vector_t&)>;
  using ad_parameterized_function_t = std::function<void(const ad_vector_t&, const ad_vector_t&, ad_vector_t&)>;
  using ad_fun_t = CppAD::ADFun<ad_base_t>;

  /**
   * Constructor for parameterized functions
   *
   * @param adFunction : parameterized function y = f(x,p)
   * @param variableDim : size of x
   * @param parameterDim : size of p
   * @param modelName : Name of the library to be generated.
   * @param folderName : Folder to save library files to, either absolute of relative
   * @param compileFlags : Compilation flags for the model library.
   */
  CppAdInterface(ad_parameterized_function_t adFunction, size_t variableDim, size_t parameterDim, std::string modelName,
                 std::string folderName = "/tmp/ocs2",
                 std::vector<std::string> compileFlags = {"-O3", "-g", "-march=native", "-mtune=native", "-ffast-math"});

  /**
   * Constructor for functions without parameters
   *
   * @param adFunction : function y = f(x)
   * @param variableDim : size of x
   * @param modelName : Name of the library to be generated.
   * @param folderName : Folder to save library files to, either absolute of relative
   * @param compileFlags : Compilation flags for the model library.
   */
  CppAdInterface(ad_function_t adFunction, size_t variableDim, std::string modelName, std::string folderName = "/tmp/ocs2",
                 std::vector<std::string> compileFlags = {"-O3", "-g", "-march=native", "-mtune=native", "-ffast-math"});

  ~CppAdInterface() = default;

  /**
   * Copy constructor. Models are reloaded if available.
   */
  CppAdInterface(const CppAdInterface& rhs);

  //! Not implemented
  CppAdInterface& operator=(const CppAdInterface& rhs) = delete;
  CppAdInterface(CppAdInterface&& rhs) = delete;
  CppAdInterface& operator=(CppAdInterface&& rhs) = delete;

  /**
   * Loads earlier created model from disk
   */
  void loadModels(bool verbose = true);

  /**
   * Creates models, compiles them, and saves them to disk
   *
   * @param approximationOrder : Order of derivatives to generate
   * @param verbose : Print out extra information
   */
  void createModels(ApproximationOrder approximationOrder = ApproximationOrder::Second, bool verbose = true);

  /**
   * Load models if they are available on disk. Creates a new library otherwise.
   *
   * @param approximationOrder : Order of derivatives to generate
   * @param verbose : Print out extra information
   */
  void loadModelsIfAvailable(ApproximationOrder approximationOrder = ApproximationOrder::Second, bool verbose = true);

  /**
   * @param x : input vector of size variableDim
   * @param p : parameter vector of size parameterDim
   * @return y = f(x,p)
   */
  vector_t getFunctionValue(const vector_t& x, const vector_t& p = vector_t(0)) const;

  /**
   * Jacobian with gradient of each output w.r.t the variables x in the rows.
   *
   * @param x : input vector of size variableDim
   * @param p : parameter vector of size parameterDim
   * @return d/dx( f(x,p) )
   */
  matrix_t getJacobian(const vector_t& x, const vector_t& p = vector_t(0)) const;

  /**
   * Returns the full Gauss-Newton approximation of the function.
   * With auto differentiated function y = f(x,p), the following approximation is made:
   *
   * value (scalar) : 0.5 * |y|^2
   * first derivative : dy/dx' * y
   * second derivative : dy/dx' * dy/dx
   *
   * @param x : input vector of size variableDim
   * @param p : parameter vector of size parameterDim
   * @return Quadratic approximation with the values stored in f, dfdx, dfdxx.
   */
  ScalarFunctionQuadraticApproximation getGaussNewtonApproximation(const vector_t& x, const vector_t& p = vector_t(0)) const;

  /**
   * Hessian, available per output.
   *
   * @param outputIndex : Output to get the hessian for.
   * @param x : input vector of size variableDim
   * @param p : parameter vector of size parameterDim
   * @return dd/dxdx( f_i(x,p) )
   */
  matrix_t getHessian(size_t outputIndex, const vector_t& x, const vector_t& p = vector_t(0)) const;

  /**
   * Weighted hessian
   *
   * @param w: vector of weights of size rangeDim
   * @param x : input vector of size variableDim
   * @param p : parameter vector of size parameterDim
   * @return dd/dxdx(sum_i  w_i*f_i(x,p) )
   */
  matrix_t getHessian(const vector_t& w, const vector_t& x, const vector_t& p = vector_t(0)) const;

 private:
  /**
   * Defines library folder names
   */
  void setFolderNames();

  /**
   * Creates folders on disk
   */
  void createFolderStructure() const;

  /**
   * Checks if library can already be found on disk.
   * @return isLibraryAvailable
   */
  bool isLibraryAvailable() const;

  /**
   * Creates a random temporary folder name
   * @return folder name
   */
  std::string getUniqueTemporaryName() const;

  /**
   * Configures the compiler that compiles the model library
   * @param compiler : compiler to be configured
   */
  void setCompilerOptions(CppAD::cg::GccCompiler<scalar_t>& compiler) const;

  /**
   * Configure the approximation order for the source generator
   * @param approximationOrder
   * @param sourceGen
   * @param fun
   */
  void setApproximationOrder(ApproximationOrder approximationOrder, CppAD::cg::ModelCSourceGen<scalar_t>& sourceGen, ad_fun_t& fun) const;

  /**
   * Stores the sparisty nonzeros
   */
  void setSparsityNonzeros();

  /**
   * Creates sparsity pattern for the Jacobian that will be generated
   * @param fun : taped ad function
   * @return Sparsity pattern that contains entries for variables only, not for parameters
   */
  cppad_sparsity::SparsityPattern createJacobianSparsity(ad_fun_t& fun) const;

  /**
   * Creates sparsity pattern for the Hessian that will be generated
   * @param fun : taped ad function
   * @return Sparsity pattern that contains entries for variables only, not for parameters
   */
  cppad_sparsity::SparsityPattern createHessianSparsity(ad_fun_t& fun) const;

  std::unique_ptr<CppAD::cg::DynamicLib<scalar_t>> dynamicLib_;
  std::unique_ptr<CppAD::cg::GenericModel<scalar_t>> model_;
  ad_parameterized_function_t adFunction_;
  std::vector<std::string> compileFlags_;

  // Sizes
  size_t variableDim_;
  size_t parameterDim_;
  size_t rangeDim_ = 0;
  size_t nnzJacobian_ = 0;
  size_t nnzHessian_ = 0;

  // Names
  std::string modelName_;
  std::string folderName_;
  std::string libraryFolder_;
  std::string tmpName_;
  std::string tmpFolder_;
  std::string libraryName_;
};

}  // namespace ocs2
