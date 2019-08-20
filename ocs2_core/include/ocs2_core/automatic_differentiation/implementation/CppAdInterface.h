//
// Created by rgrandia on 19.08.19.
//

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
    : CppAdInterface([=](const ad_dynamic_vector_t& x, const ad_dynamic_vector_t& p, ad_dynamic_vector_t& y) { adFunction(x, y); },
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

  // Compiler objects
  CppAD::cg::ModelLibraryCSourceGen<scalar_t> libraryCSourceGen(sourceGen);
  CppAD::cg::GccCompiler<scalar_t> gccCompiler;
  CppAD::cg::DynamicModelLibraryProcessor<scalar_t> libraryProcessor(libraryCSourceGen, libraryName_);
  setCompilerOptions(gccCompiler);

  if (verbose) {
    std::cerr << "Compiling Shared Library: " << libraryProcessor.getLibraryName() + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION
              << std::endl;
  }

  // Compile and store the library
  dynamicLib_.reset(libraryProcessor.createDynamicLibrary(gccCompiler));
  model_.reset(dynamicLib_->model(modelName_));
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
  model_.reset(dynamicLib_->model(modelName_));
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

  model_->ForwardZero(xp.data(), xp.size(), functionValue.data(), functionValue.size());
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

  // Call SparseJacobian with std::vector. Other CppAd functions allocate internal vectors that are incompatible with multithreading.
  std::vector<scalar_t> xp_v(xp.data(), xp.data() + xp.size());
  std::vector<scalar_t> sparseJacobian;
  std::vector<size_t> rows;
  std::vector<size_t> cols;
  model_->SparseJacobian(xp_v, sparseJacobian, rows, cols);

  // Write sparse elements into Eigen type. Only jacobian w.r.t. variables was requested, so cols should not contain elements corresponding
  // to parameters
  dynamic_matrix_t jacobian = dynamic_matrix_t::Zero(rangeDim_, variableDim_);
  int numNonZeros = rows.size();
  for (int i = 0; i < numNonZeros; ++i) {
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
  assert(w.size() == rangeDim_);
  dynamic_vector_t xp(variableDim_ + parameterDim_);
  xp << x, p;

  // Call SparseHessian with std::vector. Other CppAd functions allocate internal vectors that are incompatible with multithreading.
  std::vector<scalar_t> xp_v(xp.data(), xp.data() + xp.size());
  std::vector<scalar_t> w_v(w.data(), w.data() + w.size());
  std::vector<scalar_t> sparseHessian;
  std::vector<size_t> rows;
  std::vector<size_t> cols;
  model_->SparseHessian(xp_v, w_v, sparseHessian, rows, cols);

  // Fills upper triangular sparsity of hessian w.r.t variables.
  dynamic_matrix_t hessian = dynamic_matrix_t::Zero(variableDim_, variableDim_);
  int numNonZeros = rows.size();
  for (int i = 0; i < numNonZeros; ++i) {
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
  tmpFolder_ = libraryFolder_ + "/" + getUniqueTemporaryFolderName();
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
std::string CppAdInterface<scalar_t>::getUniqueTemporaryFolderName() const {
  // Add random string to tmp folder to avoid race condition on the temporary objects
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