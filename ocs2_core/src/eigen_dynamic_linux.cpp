/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */

#include <Eigen/Core>
#include <cppad/cg.hpp>

template <size_t NUM_ROWS, size_t NUM_COLS>
void extractSparsityPattern(const Eigen::Matrix<bool, NUM_ROWS, NUM_COLS>& sparsityPattern, std::vector<size_t>& row,
                            std::vector<size_t>& col) {
  row.clear();
  col.clear();

  for (size_t i = 0; i < NUM_ROWS; i++)
    for (size_t j = 0; j < NUM_COLS; j++)
      if (sparsityPattern(i, j) == true) {
        row.push_back(i);
        col.push_back(j);
      }
}

int main(void) {
  typedef double scalar_t;
  typedef CppAD::cg::CG<scalar_t> ad_base_t;
  typedef CppAD::AD<ad_base_t> ad_t;
  typedef CppAD::ADFun<ad_base_t> ad_fun_t;

  const size_t IN_DIM = 3;
  const size_t OUT_DIM = 2;

  /***************************************************************************
   *                               the model
   **************************************************************************/

  // set and declare independent variables and start tape recording
  Eigen::Matrix<ad_t, Eigen::Dynamic, 1> x(IN_DIM);
  x.setZero();
  CppAD::Independent(x);

  // dependent variable vector
  Eigen::Matrix<ad_t, Eigen::Dynamic, 1> y(OUT_DIM);

  // the model equation
  y(0) = (x(0) / 1.0 + x(1) * x(1)) / 2.0 + -5.0 * x(2);
  y(1) = -2.0 * x(0) - 3.0 * x(1) + -4.0 * x(2);

  // create f: x -> y and stop tape recording
  ad_fun_t fun(x, y);

  // Optimize the operation sequence
  fun.optimize();

  /***************************************************************************
   *                              Sparsity Pattern
   **************************************************************************/
  Eigen::Matrix<bool, OUT_DIM, IN_DIM> sparsityPattern;
  sparsityPattern.setOnes();
  sparsityPattern.col(2).setZero();
  std::cout << "sparsityPattern: \n" << sparsityPattern << std::endl;

  std::vector<size_t> row;
  std::vector<size_t> col;
  extractSparsityPattern<OUT_DIM, IN_DIM>(sparsityPattern, row, col);

  /***************************************************************************
   *                       Create the dynamic library
   *                  (generates and compiles source code)
   **************************************************************************/
  // generates source code
  CppAD::cg::ModelCSourceGen<scalar_t> cgen(fun, "model");
  cgen.setCreateJacobian(true);
  cgen.setCreateHessian(true);
  //    cgen.setCreateForwardOne(true);
  //    cgen.setCreateReverseOne(true);
  //    cgen.setCreateReverseTwo(true);
  cgen.setCustomSparseJacobianElements(row, col);
  cgen.setCreateSparseJacobian(true);

  CppAD::cg::ModelLibraryCSourceGen<scalar_t> libcgen(cgen);

  // compile source code
  const std::string libraryName = "cppad_cg_model";
  CppAD::cg::DynamicModelLibraryProcessor<scalar_t> p(libcgen, libraryName);

  CppAD::cg::GccCompiler<scalar_t> compiler;
  std::unique_ptr<CppAD::cg::DynamicLib<scalar_t>> dynamicLib(p.createDynamicLibrary(compiler));

  // save to files (not really required)
  CppAD::cg::SaveFilesModelLibraryProcessor<scalar_t> p2(libcgen);
  p2.saveSourcesTo("cppad_cg_model_folder");

  /***************************************************************************
   *                       Use the dynamic library
   **************************************************************************/

  std::unique_ptr<CppAD::cg::GenericModel<scalar_t>> model(dynamicLib->model("model"));

  Eigen::Matrix<scalar_t, 1, Eigen::Dynamic, Eigen::RowMajor> xv(IN_DIM);
  xv << 2.5, 3.5, 0.0;

  if (model->isJacobianAvailable()) {
    Eigen::Matrix<scalar_t, 1, Eigen::Dynamic, Eigen::RowMajor> jac(IN_DIM * OUT_DIM);
    model->Jacobian(xv, jac);

    Eigen::Matrix<scalar_t, OUT_DIM, IN_DIM> jacMat = Eigen::Map<Eigen::Matrix<scalar_t, OUT_DIM, IN_DIM, Eigen::RowMajor>>(jac.data());

    // print out the result
    std::cout << "jac: " << std::endl;
    std::cout << jac << std::endl;

    std::cout << "jacMat: " << std::endl;
    std::cout << jacMat << std::endl;
  }

  if (model->isJacobianAvailable()) {
    typedef Eigen::Matrix<scalar_t, 1, Eigen::Dynamic, Eigen::RowMajor> dynamic_vector_t;

    const Eigen::Matrix<scalar_t, IN_DIM, 1> x(xv);
    Eigen::Map<dynamic_vector_t> xvv(const_cast<scalar_t*>(x.data()), IN_DIM);

    Eigen::Matrix<scalar_t, IN_DIM, OUT_DIM> jacMat;
    Eigen::Map<dynamic_vector_t> jac(jacMat.data(), OUT_DIM * IN_DIM);

    model->Jacobian(xvv, jac);

    // print out the result
    std::cout << "jacMat2: " << std::endl;
    std::cout << jacMat.transpose() << std::endl;
  }

  if (model->isSparseJacobianAvailable()) {
    Eigen::Matrix<scalar_t, 1, Eigen::Dynamic, Eigen::RowMajor> jacSparce(IN_DIM * OUT_DIM);
    model->SparseJacobian(xv, jacSparce);

    Eigen::Matrix<scalar_t, OUT_DIM, IN_DIM> jacSparceMat =
        Eigen::Map<Eigen::Matrix<scalar_t, OUT_DIM, IN_DIM, Eigen::RowMajor>>(jacSparce.data());

    // print out the result
    std::cout << "jacSparceMat: " << std::endl;
    std::cout << jacSparceMat << std::endl;
  }

  if (model->isForwardZeroAvailable() == true) {
    Eigen::Matrix<scalar_t, 1, Eigen::Dynamic, Eigen::RowMajor> fancValue;
    model->ForwardZero(xv, fancValue);
    std::cout << "Function: " << std::endl;
    std::cout << fancValue << std::endl;
  }

  if (model->isHessianAvailable()) {
    Eigen::Matrix<scalar_t, 1, Eigen::Dynamic, Eigen::RowMajor> weight(OUT_DIM);
    Eigen::Matrix<scalar_t, 1, Eigen::Dynamic, Eigen::RowMajor> hess(IN_DIM * IN_DIM);
    Eigen::Matrix<scalar_t, IN_DIM, IN_DIM> hessMat;

    for (size_t i = 0; i < OUT_DIM; i++) {
      weight.setZero();
      weight(i) = 1.0;

      model->Hessian(xv, weight, hess);

      hessMat = Eigen::Map<Eigen::Matrix<scalar_t, IN_DIM, IN_DIM, Eigen::RowMajor>>(hess.data());

      // print out the result
      std::cout << "hessMat[" << i << "]: " << std::endl;
      std::cout << hessMat << std::endl;
    }
  }
}
