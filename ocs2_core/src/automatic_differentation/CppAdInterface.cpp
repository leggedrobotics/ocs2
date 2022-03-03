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

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

#include <boost/filesystem.hpp>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CppAdInterface::CppAdInterface(ad_parameterized_function_t adFunction, size_t variableDim, size_t parameterDim, std::string modelName,
                               std::string folderName, std::vector<std::string> compileFlags)
    : adFunction_(std::move(adFunction)),
      variableDim_(variableDim),
      parameterDim_(parameterDim),
      modelName_(std::move(modelName)),
      folderName_(std::move(folderName)),
      compileFlags_(std::move(compileFlags)) {
  setFolderNames();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CppAdInterface::CppAdInterface(ad_function_t adFunction, size_t variableDim, std::string modelName, std::string folderName,
                               std::vector<std::string> compileFlags)
    : CppAdInterface([adFunction](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) { adFunction(x, y); }, variableDim, 0,
                     std::move(modelName), std::move(folderName), std::move(compileFlags)){};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CppAdInterface::CppAdInterface(const CppAdInterface& rhs)
    : CppAdInterface(rhs.adFunction_, rhs.variableDim_, rhs.parameterDim_, rhs.modelName_, rhs.folderName_, rhs.compileFlags_) {
  if (isLibraryAvailable()) {
    loadModels(false);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CppAdInterface::createModels(ApproximationOrder approximationOrder, bool verbose) {
  createFolderStructure();

  // set and declare independent variables and start tape recording
  ad_vector_t xp(variableDim_ + parameterDim_);
  xp.setOnes();  // Ones are better than zero, to prevent devision by zero in taping
  CppAD::Independent(xp);

  // Split in variables and parameters
  ad_vector_t x = xp.segment(0, variableDim_);
  ad_vector_t p = xp.segment(variableDim_, parameterDim_);
  // dependent variable vector
  ad_vector_t y;
  // the model equation
  adFunction_(x, p, y);
  rangeDim_ = y.rows();
  // create f: xp -> y and stop tape recording
  ad_fun_t fun(xp, y);
  // Optimize the operation sequence
  fun.optimize();

  // generates source code
  CppAD::cg::ModelCSourceGen<scalar_t> sourceGen(fun, modelName_);
  setApproximationOrder(approximationOrder, sourceGen, fun);

  // Compiler objects, compile to temporary shared library file to avoid interference between processes
  CppAD::cg::ModelLibraryCSourceGen<scalar_t> libraryCSourceGen(sourceGen);
  CppAD::cg::GccCompiler<scalar_t> gccCompiler;
  CppAD::cg::DynamicModelLibraryProcessor<scalar_t> libraryProcessor(libraryCSourceGen, libraryName_ + tmpName_);
  setCompilerOptions(gccCompiler);

  if (verbose) {
    std::cerr << "[CppAdInterface] Compiling Shared Library: "
              << libraryName_ + tmpName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION << std::endl;
  }

  // Compile and store the library
  dynamicLib_ = libraryProcessor.createDynamicLibrary(gccCompiler);
  model_ = dynamicLib_->model(modelName_);

  setSparsityNonzeros();

  // Rename generated library after loading
  if (verbose) {
    std::cerr << "[CppAdInterface] Renaming " << libraryName_ + tmpName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION << " to "
              << libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION << std::endl;
  }
  boost::filesystem::rename(libraryName_ + tmpName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION,
                            libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CppAdInterface::loadModels(bool verbose) {
  if (verbose) {
    std::cerr << "[CppAdInterface] Loading Shared Library: " << libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION
              << std::endl;
  }
  dynamicLib_.reset(new CppAD::cg::LinuxDynamicLib<scalar_t>(libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION));
  model_ = dynamicLib_->model(modelName_);
  rangeDim_ = model_->Range();

  setSparsityNonzeros();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CppAdInterface::loadModelsIfAvailable(ApproximationOrder approximationOrder, bool verbose) {
  if (isLibraryAvailable()) {
    loadModels(verbose);
  } else {
    createModels(approximationOrder, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CppAdInterface::getFunctionValue(const vector_t& x, const vector_t& p) const {
  vector_t xp(variableDim_ + parameterDim_);
  xp << x, p;

  vector_t functionValue(model_->Range());

  model_->ForwardZero(xp, functionValue);
  assert(functionValue.allFinite());
  return functionValue;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t CppAdInterface::getJacobian(const vector_t& x, const vector_t& p) const {
  // Concatenate input
  vector_t xp(variableDim_ + parameterDim_);
  xp << x, p;
  CppAD::cg::ArrayView<scalar_t> xpArrayView(xp.data(), xp.size());

  std::vector<scalar_t> sparseJacobian(nnzJacobian_);
  CppAD::cg::ArrayView<scalar_t> sparseJacobianArrayView(sparseJacobian);
  size_t const* rows;
  size_t const* cols;
  // Call this particular SparseJacobian. Other CppAd functions allocate internal vectors that are incompatible with multithreading.
  model_->SparseJacobian(xpArrayView, sparseJacobianArrayView, &rows, &cols);

  // Write sparse elements into Eigen type. Only jacobian w.r.t. variables was requested, so cols should not contain elements corresponding
  // to parameters.
  matrix_t jacobian = matrix_t::Zero(model_->Range(), variableDim_);
  for (size_t i = 0; i < nnzJacobian_; i++) {
    jacobian(rows[i], cols[i]) = sparseJacobian[i];
  }

  assert(jacobian.allFinite());
  return jacobian;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation CppAdInterface::getGaussNewtonApproximation(const vector_t& x, const vector_t& p) const {
  // Concatenate input
  vector_t xp(variableDim_ + parameterDim_);
  xp << x, p;
  CppAD::cg::ArrayView<scalar_t> xpArrayView(xp.data(), xp.size());

  ScalarFunctionQuadraticApproximation gnApprox;

  // Zero order
  vector_t valueVector(model_->Range());
  model_->ForwardZero(xp, valueVector);
  gnApprox.f = 0.5 * valueVector.squaredNorm();

  // Jacobian
  std::vector<scalar_t> sparseJacobian(nnzJacobian_);
  CppAD::cg::ArrayView<scalar_t> sparseJacobianArrayView(sparseJacobian);
  size_t const* rows;
  size_t const* cols;
  model_->SparseJacobian(xpArrayView, sparseJacobianArrayView, &rows, &cols);

  // Sparse evaluation of J' * f
  gnApprox.dfdx.setZero(variableDim_);
  for (size_t i = 0; i < nnzJacobian_; i++) {
    gnApprox.dfdx(cols[i]) += sparseJacobian[i] * valueVector(rows[i]);
  }

  /*
   * Sparse construction of the GN matrix, H = J' * J.
   * H(i, j) = sum_rows { J(row, i) * J(row, j) }
   * Because the sparse elements are ordered first by row, then by column, we process J row-by-row.
   * For each row of J, we add the non-zero pairs (i, j) to H(i, j).
   */
  gnApprox.dfdxx.setZero(variableDim_, variableDim_);
  for (size_t i = 0; i < nnzJacobian_; ++i) {
    const size_t row_i = rows[i];
    const size_t col_i = cols[i];
    const scalar_t v_i = sparseJacobian[i];
    // Diagonal element always exists:
    gnApprox.dfdxx(col_i, col_i) += v_i * v_i;
    // Process off-diagonals
    size_t j = i + 1;
    while (rows[j] == row_i) {
      const size_t col_j = cols[j];
      gnApprox.dfdxx(col_j, col_i) += v_i * sparseJacobian[j];
      gnApprox.dfdxx(col_i, col_j) = gnApprox.dfdxx(col_j, col_i);  // Maintain symmetry as we go.
      ++j;
    }
  }

  assert(gnApprox.dfdx.allFinite());
  assert(gnApprox.dfdxx.allFinite());
  return gnApprox;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t CppAdInterface::getHessian(size_t outputIndex, const vector_t& x, const vector_t& p) const {
  vector_t w = vector_t::Zero(rangeDim_);
  w[outputIndex] = 1.0;

  return getHessian(w, x, p);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t CppAdInterface::getHessian(const vector_t& w, const vector_t& x, const vector_t& p) const {
  // Concatenate input
  vector_t xp(variableDim_ + parameterDim_);
  xp << x, p;
  CppAD::cg::ArrayView<const scalar_t> xpArrayView(xp.data(), xp.size());

  std::vector<scalar_t> sparseHessian(nnzHessian_);
  CppAD::cg::ArrayView<scalar_t> sparseHessianArrayView(sparseHessian);
  size_t const* rows;
  size_t const* cols;

  CppAD::cg::ArrayView<const scalar_t> wArrayView(w.data(), w.size());

  // Call this particular SparseHessian. Other CppAd functions allocate internal vectors that are incompatible with multithreading.
  model_->SparseHessian(xpArrayView, wArrayView, sparseHessianArrayView, &rows, &cols);

  // Fills upper triangular sparsity of hessian w.r.t variables.
  matrix_t hessian = matrix_t::Zero(variableDim_, variableDim_);
  for (size_t i = 0; i < nnzHessian_; i++) {
    hessian(rows[i], cols[i]) = sparseHessian[i];
  }

  // Copy upper triangular to lower triangular part
  hessian.template triangularView<Eigen::StrictlyLower>() = hessian.template triangularView<Eigen::StrictlyUpper>().transpose();

  assert(hessian.allFinite());
  return hessian;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CppAdInterface::setFolderNames() {
  if (!folderName_.empty()) {
    libraryFolder_ = folderName_ + "/" + modelName_ + "/cppad_generated";
  } else {
    libraryFolder_ = modelName_ + "/cppad_generated";
  }
  tmpName_ = getUniqueTemporaryName();
  tmpFolder_ = libraryFolder_ + "/" + tmpName_;
  libraryName_ = libraryFolder_ + "/" + modelName_ + "_lib";
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool CppAdInterface::isLibraryAvailable() const {
  return boost::filesystem::exists(libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CppAdInterface::createFolderStructure() const {
  boost::filesystem::create_directories(libraryFolder_);
  boost::filesystem::create_directories(tmpFolder_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string CppAdInterface::getUniqueTemporaryName() const {
  // Random string should be unique for each process and time of calling.
  int randomFromClock = std::chrono::high_resolution_clock::now().time_since_epoch().count() % 1000;
  return std::string("cppadcg_tmp") + std::to_string(randomFromClock) + std::to_string(getpid());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CppAdInterface::setCompilerOptions(CppAD::cg::GccCompiler<scalar_t>& compiler) const {
  if (!compileFlags_.empty()) {
    // Set compile flags and add required flags for dynamic compilation
    auto compileFlags = compileFlags_;
    compiler.setCompileLibFlags(compileFlags_);
    compiler.addCompileLibFlag("-shared");
    compiler.addCompileLibFlag("-rdynamic");
  }

  compiler.setTemporaryFolder(tmpFolder_);

  // Save sources
  compiler.setSourcesFolder(libraryFolder_);
  compiler.setSaveToDiskFirst(true);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CppAdInterface::setApproximationOrder(ApproximationOrder approximationOrder, CppAD::cg::ModelCSourceGen<scalar_t>& sourceGen,
                                           ad_fun_t& fun) const {
  switch (approximationOrder) {
    case ApproximationOrder::Second:
      sourceGen.setCreateSparseHessian(true);
      sourceGen.setCustomSparseHessianElements(createHessianSparsity(fun));
      // Intentional fall through
    case ApproximationOrder::First:
      sourceGen.setCreateSparseJacobian(true);
      sourceGen.setCustomSparseJacobianElements(createJacobianSparsity(fun));
      // Intentional fall through
    case ApproximationOrder::Zero:
      break;
    default:
      throw std::runtime_error("CppAdInterface: Invalid approximation order");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CppAdInterface::setSparsityNonzeros() {
  if (model_->isJacobianSparsityAvailable()) {
    nnzJacobian_ = cppad_sparsity::getNumberOfNonZeros(model_->JacobianSparsitySet());
  }
  if (model_->isHessianSparsityAvailable()) {
    nnzHessian_ = cppad_sparsity::getNumberOfNonZeros(model_->HessianSparsitySet());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
cppad_sparsity::SparsityPattern CppAdInterface::createJacobianSparsity(ad_fun_t& fun) const {
  auto trueSparsity = cppad_sparsity::getJacobianSparsityPattern(fun);
  auto variableSparsity = cppad_sparsity::getJacobianVariableSparsity(rangeDim_, variableDim_);
  return cppad_sparsity::getIntersection(trueSparsity, variableSparsity);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
cppad_sparsity::SparsityPattern CppAdInterface::createHessianSparsity(ad_fun_t& fun) const {
  auto trueSparsity = cppad_sparsity::getHessianSparsityPattern(fun);
  auto variableSparsity = cppad_sparsity::getHessianVariableSparsity(variableDim_, parameterDim_);
  return cppad_sparsity::getIntersection(trueSparsity, variableSparsity);
}

}  // namespace ocs2
