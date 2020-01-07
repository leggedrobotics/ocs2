

#pragma once

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

#include <boost/filesystem.hpp>

namespace ocs2 {

template <typename scalar_t>
CppAdInterface<scalar_t>::CppAdInterface(ad_parameterized_function_t adFunction, int rangeDim, int variableDim, int parameterDim,
                                         std::string modelName, std::string folderName, std::vector<std::string> compileFlags)
    : adFunction_(std::move(adFunction)),
      rangeDim_(rangeDim),
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
template <typename scalar_t>
CppAdInterface<scalar_t>::CppAdInterface(ad_function_t adFunction, int rangeDim, int variableDim, std::string modelName,
                                         std::string folderName, std::vector<std::string> compileFlags)
    : CppAdInterface([adFunction](const ad_dynamic_vector_t& x, const ad_dynamic_vector_t& p, ad_dynamic_vector_t& y) { adFunction(x, y); },
                     rangeDim, variableDim, 0, std::move(modelName), std::move(folderName), std::move(compileFlags)){};

template <typename scalar_t>
CppAdInterface<scalar_t>::CppAdInterface(const CppAdInterface& rhs)
    : CppAdInterface(rhs.adFunction_, rhs.rangeDim_, rhs.variableDim_, rhs.parameterDim_, rhs.modelName_, rhs.folderName_,
                     rhs.compileFlags_) {
  if (isLibraryAvailable()) {
    loadModels(false);
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t>
void CppAdInterface<scalar_t>::createModels(ApproximationOrder approximationOrder, bool verbose) {
  createFolderStructure();

  // set and declare independent variables and start tape recording
  ad_dynamic_vector_t xp(variableDim_ + parameterDim_);
  xp.setOnes();  // Ones are better than zero, to prevent devision by zero in taping
  CppAD::Independent(xp);

  // Split in variables and parameters
  ad_dynamic_vector_t x = xp.segment(0, variableDim_);
  ad_dynamic_vector_t p = xp.segment(variableDim_, parameterDim_);
  // dependent variable vector
  ad_dynamic_vector_t y(rangeDim_);
  // the model equation
  adFunction_(x, p, y);
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
    std::cerr << "Compiling Shared Library: " << libraryName_ + tmpName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION
              << std::endl;
  }

  // Compile and store the library
  dynamicLib_ = libraryProcessor.createDynamicLibrary(gccCompiler);
  model_ = dynamicLib_->model(modelName_);

  setSparsityNonzeros();

  // Rename generated library after loading
  if (verbose) {
    std::cerr << "Renaming " << libraryName_ + tmpName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION << " to "
              << libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION << std::endl;
  }
  boost::filesystem::rename(libraryName_ + tmpName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION,
                            libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t>
void CppAdInterface<scalar_t>::loadModels(bool verbose) {
  if (verbose) {
    std::cerr << "Loading Shared Library: " << libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION << std::endl;
  }
  dynamicLib_.reset(new CppAD::cg::LinuxDynamicLib<scalar_t>(libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION));
  model_ = dynamicLib_->model(modelName_);

  setSparsityNonzeros();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t>
void CppAdInterface<scalar_t>::loadModelsIfAvailable(ApproximationOrder approximationOrder, bool verbose) {
  if (isLibraryAvailable()) {
    loadModels(verbose);
  } else {
    createModels(approximationOrder, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t>
typename CppAdInterface<scalar_t>::dynamic_vector_t CppAdInterface<scalar_t>::getFunctionValue(const dynamic_vector_t& x,
                                                                                               const dynamic_vector_t& p) const {
  dynamic_vector_t xp(variableDim_ + parameterDim_);
  xp << x, p;

  dynamic_vector_t functionValue(rangeDim_);

  model_->ForwardZero(xp, functionValue);
  assert(functionValue.allFinite());
  return functionValue;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t>
typename CppAdInterface<scalar_t>::dynamic_matrix_t CppAdInterface<scalar_t>::getJacobian(const dynamic_vector_t& x,
                                                                                          const dynamic_vector_t& p) const {
  // Concatenate input
  dynamic_vector_t xp(variableDim_ + parameterDim_);
  xp << x, p;
  CppAD::cg::ArrayView<scalar_t> xpArrayView(xp.data(), xp.size());

  std::vector<scalar_t> sparseJacobian(nnzJacobian_);
  CppAD::cg::ArrayView<scalar_t> sparseJacobianArrayView(sparseJacobian);
  size_t const* rows;
  size_t const* cols;
  // Call this particular SparseJacobian. Other CppAd functions allocate internal vectors that are incompatible with multithreading.
  model_->SparseJacobian(xpArrayView, sparseJacobianArrayView, &rows, &cols);

  // Write sparse elements into Eigen type. Only jacobian w.r.t. variables was requested, so cols should not contain elements corresponding
  // to parameters. Write to rowMajor type because sparsity is specified as row major.
  dynamic_rowMajor_matrix_t jacobian = dynamic_rowMajor_matrix_t::Zero(rangeDim_, variableDim_);
  for (size_t i = 0; i < nnzJacobian_; i++) {
    jacobian(rows[i], cols[i]) = sparseJacobian[i];
  }

  assert(jacobian.allFinite());
  return jacobian;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t>
typename CppAdInterface<scalar_t>::dynamic_matrix_t CppAdInterface<scalar_t>::getHessian(size_t outputIndex, const dynamic_vector_t& x,
                                                                                         const dynamic_vector_t& p) const {
  dynamic_vector_t w = dynamic_vector_t::Zero(rangeDim_);
  w[outputIndex] = 1.0;

  return getHessian(w, x, p);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t>
typename CppAdInterface<scalar_t>::dynamic_matrix_t CppAdInterface<scalar_t>::getHessian(const dynamic_vector_t& w,
                                                                                         const dynamic_vector_t& x,
                                                                                         const dynamic_vector_t& p) const {
  // Concatenate input
  dynamic_vector_t xp(variableDim_ + parameterDim_);
  xp << x, p;
  CppAD::cg::ArrayView<const scalar_t> xpArrayView(xp.data(), xp.size());

  std::vector<scalar_t> sparseHessian(nnzHessian_);
  CppAD::cg::ArrayView<scalar_t> sparseHessianArrayView(sparseHessian);
  size_t const* rows;
  size_t const* cols;

  CppAD::cg::ArrayView<const scalar_t> wArrayView(w.data(), w.size());

  // Call this particular SparseHessian. Other CppAd functions allocate internal vectors that are incompatible with multithreading.
  model_->SparseHessian(xpArrayView, wArrayView, sparseHessianArrayView, &rows, &cols);

  // Fills upper triangular sparsity of hessian w.r.t variables. Write to rowMajor type because sparsity is specified as row major.
  dynamic_rowMajor_matrix_t hessian = dynamic_rowMajor_matrix_t::Zero(variableDim_, variableDim_);
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
template <typename scalar_t>
void CppAdInterface<scalar_t>::setFolderNames() {
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
template <typename scalar_t>
bool CppAdInterface<scalar_t>::isLibraryAvailable() const {
  return boost::filesystem::exists(libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t>
void CppAdInterface<scalar_t>::createFolderStructure() const {
  boost::filesystem::create_directories(libraryFolder_);
  boost::filesystem::create_directories(tmpFolder_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t>
std::string CppAdInterface<scalar_t>::getUniqueTemporaryName() const {
  // Random string should be unique for each process and time of calling.
  int randomFromClock = std::chrono::high_resolution_clock::now().time_since_epoch().count() % 1000;
  return std::string("cppadcg_tmp") + std::to_string(randomFromClock) + std::to_string(getpid());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t>
void CppAdInterface<scalar_t>::setCompilerOptions(CppAD::cg::GccCompiler<scalar_t>& compiler) const {
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
template <typename scalar_t>
void CppAdInterface<scalar_t>::setApproximationOrder(ApproximationOrder approximationOrder, CppAD::cg::ModelCSourceGen<scalar_t>& sourceGen,
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
template <typename scalar_t>
void CppAdInterface<scalar_t>::setSparsityNonzeros() {
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
template <typename scalar_t>
typename CppAdInterface<scalar_t>::SparsityPattern CppAdInterface<scalar_t>::createJacobianSparsity(ad_fun_t& fun) const {
  auto trueSparsity = cppad_sparsity::getJacobianSparsityPattern(fun);
  auto variableSparsity = cppad_sparsity::getJacobianVariableSparsity(rangeDim_, variableDim_);
  return cppad_sparsity::getIntersection(trueSparsity, variableSparsity);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename scalar_t>
typename CppAdInterface<scalar_t>::SparsityPattern CppAdInterface<scalar_t>::createHessianSparsity(ad_fun_t& fun) const {
  auto trueSparsity = cppad_sparsity::getHessianSparsityPattern(fun);
  auto variableSparsity = cppad_sparsity::getHessianVariableSparsity(variableDim_, parameterDim_);
  return cppad_sparsity::getIntersection(trueSparsity, variableSparsity);
}

}  // namespace ocs2