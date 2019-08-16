//
// Created by rgrandia on 14.08.19.
//

#include <functional>  // Missing include in external cg.hpp
#include <memory>

#include <cppad/cg.hpp>

#include <Eigen/Core>

template class CppAD::cg::GccCompiler<double>;
template class CppAD::cg::DynamicLib<double>;
template class CppAD::cg::GenericModel<double>;
template class CppAD::cg::LinuxDynamicLib<double>;
template class CppAD::cg::ModelCSourceGen<double>;
template class CppAD::cg::SaveFilesModelLibraryProcessor<double>;
template class CppAD::cg::DynamicModelLibraryProcessor<double>;
template class CppAD::cg::ModelLibraryCSourceGen<double>;
template void CppAD::Independent<Eigen::Matrix<CppAD::AD<CppAD::cg::CG<double>>, Eigen::Dynamic, 1>>(Eigen::Matrix<CppAD::AD<CppAD::cg::CG<double>>, Eigen::Dynamic, 1>&);
