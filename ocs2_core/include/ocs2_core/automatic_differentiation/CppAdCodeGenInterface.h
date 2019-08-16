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

#ifndef CPPADCODEGENINTERFACE_OCS2_H_
#define CPPADCODEGENINTERFACE_OCS2_H_

#include <functional>  // Missing include in external cg.hpp
#include <memory>

#include <cppad/cg.hpp>

#include <ocs2_core/automatic_differentiation/AutomaticDifferentiationBase.h>

// Explicity instantiate doubles in separate source file
extern template class CppAD::cg::GccCompiler<double>;
extern template class CppAD::cg::DynamicLib<double>;
extern template class CppAD::cg::GenericModel<double>;
extern template class CppAD::cg::LinuxDynamicLib<double>;
extern template class CppAD::cg::ModelCSourceGen<double>;
extern template class CppAD::cg::ModelLibraryCSourceGen<double>;
extern template class CppAD::cg::SaveFilesModelLibraryProcessor<double>;
extern template class CppAD::cg::DynamicModelLibraryProcessor<double>;
extern template void CppAD::Independent<Eigen::Matrix<CppAD::AD<CppAD::cg::CG<double>>, Eigen::Dynamic, 1>>(
    Eigen::Matrix<CppAD::AD<CppAD::cg::CG<double>>, Eigen::Dynamic, 1>&);

namespace ocs2 {

template <int DOMAIN_DIM, int RANGE_DIM, typename SCALAR_T = double, int VARIABLE_DIM = DOMAIN_DIM>
class CppAdCodeGenInterface : public AutomaticDifferentiationBase<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = AutomaticDifferentiationBase<DOMAIN_DIM, RANGE_DIM, SCALAR_T, VARIABLE_DIM>;
  using typename BASE::domain_matrix_t;
  using typename BASE::domain_range_matrix_t;
  using typename BASE::domain_vector_t;
  using typename BASE::range_domain_matrix_t;
  using typename BASE::range_matrix_t;
  using typename BASE::range_vector_t;
  using typename BASE::variable_matrix_t;
  using typename BASE::variable_vector_t;

  using ad_base_t = CppAD::cg::CG<SCALAR_T>;
  using ad_scalar_t = CppAD::AD<ad_base_t>;
  using ad_fun_t = CppAD::ADFun<ad_base_t>;

  using ad_dynamic_vector_t = Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, 1>;

  using ad_funtion_t = std::function<void(const ad_dynamic_vector_t&, ad_dynamic_vector_t&)>;

  CppAdCodeGenInterface(const ad_funtion_t& adFunction, const range_domain_matrix_t& sparsityPattern);

  CppAdCodeGenInterface(int domainDim, int rangeDim, int variableDim, const ad_funtion_t& adFunction,
                        const range_domain_matrix_t& sparsityPattern);

  ~CppAdCodeGenInterface() = default;

  CppAdCodeGenInterface(const CppAdCodeGenInterface& rhs);

  CppAdCodeGenInterface* clone() const override;

  void computeForwardModel(bool computeForwardModel) override;

  void computeJacobianModel(bool computeJacobianModel) override;

  void computeHessianModel(bool computeHessianModel) override;

  void createModels(const std::string& modelName, const std::string& libraryFolder = "", const bool verbose = true,
                    bool cgJIT = true) override;

  void createModels(const int& domainDim, const int& rangeDim, const std::string& modelName, const std::string& libraryFolder = "",
                    bool verbose = true, bool cgJIT = true) override;

  bool loadModels(const std::string& modelName, const std::string& libraryFolder = "", bool verbose = true) override;

  void getSparsityPattern(range_domain_matrix_t& sparsityPattern) const override;

  bool getFunctionValue(const domain_vector_t& x, range_vector_t& funcValue) override;

  bool getJacobian(const domain_vector_t& x, domain_range_matrix_t& jacobian) override;

  bool getHessian(const domain_vector_t& x, variable_matrix_t& hessian, size_t outputIndex = 0) override;

  void getCompileFlags(std::vector<std::string>& cFlags) const { cFlags = compiler_.getCompileFlags(); }

  void setCompileFlags(const std::vector<std::string>& cFlags) { compiler_.setCompileFlags(cFlags); }

  void addCompileFlag(const std::vector<std::string>& cFlags) {
    for (const std::string& flag : cFlags) {
      compiler_.addCompileFlags(flag);
    }
  }

  void getCompileLibFlags(std::vector<std::string>& cFlags) const { cFlags = compiler_.getCompileLibFlags(); }

  void setCompileLibFlags(const std::vector<std::string>& cFlags) { compiler_.setCompileLibFlags(cFlags); }

  void addCompileLibFlags(const std::vector<std::string>& cFlags) {
    for (const std::string& flag : cFlags) {
      compiler_.addCompileLibFlags(flag);
    }
  }

 private:
  using dynamic_vector_t = Eigen::Matrix<SCALAR_T, 1, Eigen::Dynamic>;
  using dynamic_vector_map_t = Eigen::Map<dynamic_vector_t>;

  ad_funtion_t adFunction_;

  range_domain_matrix_t sparsityPattern_;
  bool modelFullDerivatives_;

  bool computeForwardModel_;
  bool computeJacobianModel_;
  bool computeHessianModel_;

  std::vector<size_t> rowsJacobian_;
  std::vector<size_t> colsJacobian_;
  std::vector<size_t> rowsHessian_;
  std::vector<size_t> colsHessian_;
  std::vector<size_t> zeroCountHessian_;
  std::vector<std::set<size_t>> relatedDependent_;

  CppAD::cg::GccCompiler<SCALAR_T> compiler_;
  std::unique_ptr<CppAD::cg::DynamicLib<SCALAR_T>> dynamicLib_;
  std::unique_ptr<CppAD::cg::GenericModel<SCALAR_T>> model_;

  std::vector<SCALAR_T> hessianWeight_;
};

}  // namespace ocs2

#include "implementation/CppAdCodeGenInterface.h"

#endif /* CPPADCODEGENINTERFACE_OCS2_H_ */
